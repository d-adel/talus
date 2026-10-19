#include "scheduler.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <utility>

#if defined(_M_X64) || defined(__x86_64__)
#include <immintrin.h>
#endif

namespace talus::dynamics {

namespace {

constexpr int taskPending = 1;
constexpr int taskClaimed = 2;
constexpr int taskComplete = 3;

thread_local bool t_alongside = false;

class AlongsideScope {
public:
  AlongsideScope() : m_previous(std::exchange(t_alongside, true)) {}
  ~AlongsideScope() { t_alongside = m_previous; }

  AlongsideScope(const AlongsideScope&) = delete;
  AlongsideScope& operator=(const AlongsideScope&) = delete;

private:
  bool m_previous = false;
};

struct ForShared {
  std::atomic<uint32_t> next{0u};
  uint32_t blockCount = 0u;
  uint32_t blockSize = 0u;
  uint32_t itemCount = 0u;
  const std::function<void(uint32_t, uint32_t, uint32_t)>* callback = nullptr;
};

struct ForTask {
  ForShared* shared = nullptr;
  uint32_t worker = 0u;
};

void forTrampoline(void* context) {
  const ForTask& task = *static_cast<ForTask*>(context);
  ForShared& shared = *task.shared;

  for (;;) {
    const uint32_t block = shared.next.fetch_add(1u, std::memory_order_acq_rel);

    if (block >= shared.blockCount) {
      break;
    }

    const uint32_t begin = block * shared.blockSize;
    const uint32_t end = std::min(begin + shared.blockSize, shared.itemCount);

    (*shared.callback)(begin, end, task.worker);
  }
}

constexpr uint32_t finishedStage = 0xFFFFFFFFu;
constexpr auto spinLimit = std::chrono::microseconds(50);
constexpr auto idleSpinLimit = std::chrono::microseconds(5);

void relax() {
#if defined(_M_X64) || defined(__x86_64__)
  _mm_pause();
  _mm_pause();
#else
  std::this_thread::yield();
#endif
}

template <typename Ready>
bool spinUntil(const Ready& ready, std::chrono::microseconds limit = spinLimit) {
  const auto deadline = std::chrono::steady_clock::now() + limit;

  for (uint32_t round = 1u; !ready(); ++round) {
    relax();

    if (round % 64u == 0u && std::chrono::steady_clock::now() >= deadline) {
      return ready();
    }
  }

  return true;
}

}

Scheduler::Scheduler(uint32_t workerCount)
  : m_workerCount(std::clamp(workerCount, 1u, maxWorkers)) {
  for (uint32_t index = 1u; index < m_workerCount; ++index) {
    m_threads.emplace_back([this] { serve(); });
  }
}

Scheduler::~Scheduler() {
  {
    const std::lock_guard<std::mutex> lock(m_mutex);

    m_shutdown = true;
  }

  m_wake.notify_all();

  for (std::thread& thread : m_threads) {
    thread.join();
  }
}

uint32_t Scheduler::workerCount() const {
  return m_workerCount;
}

void* Scheduler::enqueue(TaskCallback* task, void* taskContext) {
  const uint32_t slot = m_next.fetch_add(1u, std::memory_order_acq_rel);

  if (slot >= maxTasks) {
    task(taskContext);
    return nullptr;
  }

  Task& entry = m_tasks[slot];

  entry.callback = task;
  entry.context = taskContext;
  entry.status.store(taskPending, std::memory_order_release);
  post();
  m_wake.notify_one();

  return &entry;
}

void Scheduler::finish(void* userTask) {
  if (userTask == nullptr) {
    return;
  }

  Task& task = *static_cast<Task*>(userTask);
  const auto complete = [&task] { return task.status.load(std::memory_order_acquire) == taskComplete; };

  while (!complete()) {
    if (executeOne() || spinUntil(complete)) {
      continue;
    }

    m_finishers.fetch_add(1u, std::memory_order_seq_cst);

    while (task.status.load(std::memory_order_seq_cst) == taskClaimed) {
      task.status.wait(taskClaimed, std::memory_order_seq_cst);
    }

    m_finishers.fetch_sub(1u, std::memory_order_relaxed);
  }
}

void Scheduler::reset() {
  m_next.store(0u, std::memory_order_release);
}

bool Scheduler::executeOne() {
  const uint32_t count =
    std::min(m_next.load(std::memory_order_acquire), maxTasks);

  for (uint32_t slot = 0u; slot != count; ++slot) {
    Task& task = m_tasks[slot];
    int expected = taskPending;

    if (task.status.load(std::memory_order_acquire) != taskPending) {
      continue;
    }

    if (!task.status.compare_exchange_strong(
          expected,
          taskClaimed,
          std::memory_order_acq_rel
        )) {
      continue;
    }

    task.callback(task.context);
    task.status.store(taskComplete, std::memory_order_seq_cst);

    if (m_finishers.load(std::memory_order_seq_cst) != 0u) {
      task.status.notify_all();
    }

    return true;
  }

  return false;
}

void Scheduler::post() {
  const std::lock_guard<std::mutex> lock(m_mutex);

  m_wakeups.fetch_add(1u, std::memory_order_relaxed);
}

bool Scheduler::awaitWork() {
  spinUntil([this] { return m_wakeups.load(std::memory_order_relaxed) != 0u; }, idleSpinLimit);

  std::unique_lock<std::mutex> lock(m_mutex);

  m_wake.wait(lock, [this] { return m_shutdown || m_wakeups.load(std::memory_order_relaxed) != 0u; });

  if (m_shutdown) {
    return false;
  }

  m_wakeups.fetch_sub(1u, std::memory_order_relaxed);

  return true;
}

void Scheduler::serve() {
  while (awaitWork()) {
    while (executeOne()) {
    }
  }
}

void* Workers::enqueue(
  TaskCallback* task,
  void* taskContext,
  const char* name
) const {
  if (scheduler != nullptr) {
    return scheduler->enqueue(task, taskContext);
  }

  if (hostEnqueue != nullptr) {
    return hostEnqueue(task, taskContext, hostContext, name);
  }

  task(taskContext);

  return nullptr;
}

void Workers::finish(void* userTask) const {
  if (userTask == nullptr) {
    return;
  }

  if (scheduler != nullptr) {
    scheduler->finish(userTask);
  } else if (hostFinish != nullptr) {
    hostFinish(userTask, hostContext);
  }
}

void Workers::finishRun() const {
  if (scheduler != nullptr) {
    scheduler->reset();
  }
}

Workers resolveWorkers(
  uint32_t workerCount,
  EnqueueTaskCallback* enqueueTask,
  FinishTaskCallback* finishTask,
  void* userTaskContext,
  std::shared_ptr<Scheduler>& scheduler
) {
  Workers workers;

  if (workerCount > 0u && enqueueTask != nullptr && finishTask != nullptr) {
    workers.count = std::min(workerCount, maxWorkers);
    workers.hostEnqueue = enqueueTask;
    workers.hostFinish = finishTask;
    workers.hostContext = userTaskContext;

    return workers;
  }

  if (workerCount > 1u) {
    const uint32_t count = std::min(workerCount, maxWorkers);

    if (!scheduler || scheduler->workerCount() != count) {
      scheduler = std::make_shared<Scheduler>(count);
    }

    workers.count = count;
    workers.scheduler = scheduler.get();
  }

  return workers;
}

void parallelFor(
  const Workers& workers,
  uint32_t itemCount,
  uint32_t minRange,
  const std::function<void(uint32_t begin, uint32_t end, uint32_t worker)>& callback
) {
  if (itemCount == 0u) {
    return;
  }

  const uint32_t range = std::max(minRange, 1u);
  const uint32_t maxBlockCount = 32u * workers.count;
  const uint32_t blockSize =
    itemCount <= range * maxBlockCount
      ? range
      : (itemCount + maxBlockCount - 1u) / maxBlockCount;
  const uint32_t blockCount = (itemCount + blockSize - 1u) / blockSize;
  const uint32_t taskCount = std::min(workers.count, blockCount);

  if (taskCount <= 1u || t_alongside) {
    callback(0u, itemCount, 0u);
    return;
  }

  ForShared shared;
  std::array<ForTask, maxWorkers> tasks{};
  std::array<void*, maxWorkers> handles{};

  shared.blockCount = blockCount;
  shared.blockSize = blockSize;
  shared.itemCount = itemCount;
  shared.callback = &callback;

  for (uint32_t index = 0u; index != taskCount; ++index) {
    tasks[index] = {&shared, index};
    handles[index] = workers.enqueue(&forTrampoline, &tasks[index], "parallel for");
  }

  for (uint32_t index = 0u; index != taskCount; ++index) {
    workers.finish(handles[index]);
  }

  workers.finishRun();
}


StageRunner::StageRunner(
  const Workers& workers,
  std::vector<uint32_t> blockCounts,
  std::function<void(uint32_t stage, uint32_t block, uint32_t worker)> work
) {
  reset(workers, blockCounts, std::move(work));
}

void StageRunner::reset(
  const Workers& workers,
  std::span<const uint32_t> blockCounts,
  std::function<void(uint32_t stage, uint32_t block, uint32_t worker)> work,
  Claiming claiming
) {
  m_workers = workers;
  m_claiming = claiming;
  m_work = std::move(work);
  m_blockCounts.assign(blockCounts.begin(), blockCounts.end());
  m_flagStart.resize(m_blockCounts.size() + 1u);

  size_t total = 0u;

  for (size_t stage = 0u; stage != m_blockCounts.size(); ++stage) {
    m_flagStart[stage] = static_cast<uint32_t>(total);
    total += m_blockCounts[stage];
  }

  m_flagStart[m_blockCounts.size()] = static_cast<uint32_t>(total);

  if (m_claiming == Claiming::Ranges && (total > m_flagCapacity || m_flags == nullptr)) {
    m_flagCapacity = std::max<size_t>(total, 1u);
    m_flags.reset(new std::atomic<uint32_t>[m_flagCapacity]);
  }

  if (m_blockCounts.size() > m_stageCapacity || m_done == nullptr) {
    m_stageCapacity = std::max<size_t>(m_blockCounts.size(), 1u);
    m_next.reset(new std::atomic<uint32_t>[m_stageCapacity]);
    m_done.reset(new std::atomic<uint32_t>[m_stageCapacity]);
  }

  if (m_claiming == Claiming::Ranges) {
    for (size_t flag = 0u; flag != total; ++flag) {
      m_flags[flag].store(0u, std::memory_order_relaxed);
    }
  }

  for (size_t stage = 0u; stage != m_blockCounts.size(); ++stage) {
    m_next[stage].store(0u, std::memory_order_relaxed);
    m_done[stage].store(0u, std::memory_order_relaxed);
  }
}

void StageRunner::run(const std::function<void()>& orchestrate, const std::function<void()>* alongside) {
  m_orchestrate = &orchestrate;
  m_alongside = alongside;
  m_parallel = m_workers.count > 1u && !t_alongside;

  if (!m_parallel) {
    if (alongside != nullptr) {
      const AlongsideScope scope;

      (*alongside)();
    }

    orchestrate();
    return;
  }

  m_current.store(0u, std::memory_order_relaxed);
  m_claimed.store(0u, std::memory_order_relaxed);
  m_alongsideClaimed.store(0u, std::memory_order_relaxed);

  std::array<WorkerTask, maxWorkers> tasks{};
  std::array<void*, maxWorkers> handles{};

  for (uint32_t index = 0u; index != m_workers.count; ++index) {
    tasks[index] = {this, index};
    handles[index] = m_workers.enqueue(&StageRunner::serveTask, &tasks[index], "stages");
  }

  WorkerTask caller{this, 0u};

  serveTask(&caller);

  for (uint32_t index = 0u; index != m_workers.count; ++index) {
    m_workers.finish(handles[index]);
  }

  m_workers.finishRun();
}

void StageRunner::execute(uint32_t stage) {
  const uint32_t blocks = m_blockCounts[stage];

  if (blocks == 0u) {
    return;
  }

  if (!m_parallel || blocks == 1u) {
    for (uint32_t block = 0u; block != blocks; ++block) {
      m_work(stage, block, 0u);
    }

    return;
  }

  publish(stage + 1u);
  claim(stage, 0u);
  awaitDone(stage, blocks);
}

void StageRunner::publish(uint32_t value) {
  m_current.store(value, std::memory_order_seq_cst);

  if (m_sleepers.load(std::memory_order_seq_cst) != 0u) {
    m_current.notify_all();
  }
}

uint32_t StageRunner::awaitStage(uint32_t last) {
  uint32_t value = last;

  if (spinUntil([&] { return (value = m_current.load(std::memory_order_acquire)) != last; })) {
    return value;
  }

  m_sleepers.fetch_add(1u, std::memory_order_seq_cst);

  while ((value = m_current.load(std::memory_order_seq_cst)) == last) {
    m_current.wait(last, std::memory_order_seq_cst);
  }

  m_sleepers.fetch_sub(1u, std::memory_order_relaxed);

  return value;
}

void StageRunner::awaitDone(uint32_t stage, uint32_t blocks) {
  std::atomic<uint32_t>& done = m_done[stage];

  if (spinUntil([&] { return done.load(std::memory_order_acquire) == blocks; })) {
    return;
  }

  m_waiting.store(1u, std::memory_order_seq_cst);

  for (uint32_t seen = done.load(std::memory_order_seq_cst); seen != blocks; seen = done.load(std::memory_order_seq_cst)) {
    done.wait(seen, std::memory_order_seq_cst);
  }

  m_waiting.store(0u, std::memory_order_relaxed);
}

void StageRunner::countDone(uint32_t stage, uint32_t done) {
  if (done == 0u) {
    return;
  }

  m_done[stage].fetch_add(done, std::memory_order_seq_cst);

  if (m_waiting.load(std::memory_order_seq_cst) != 0u) {
    m_done[stage].notify_one();
  }
}

void StageRunner::serveTask(void* context) {
  const WorkerTask& task = *static_cast<WorkerTask*>(context);

  task.runner->serve(task.index);
}

void StageRunner::serve(uint32_t worker) {
  if (worker == 0u) {
    uint32_t expected = 0u;

    if (!m_claimed.compare_exchange_strong(
          expected,
          1u,
          std::memory_order_acq_rel
        )) {
      return;
    }

    (*m_orchestrate)();
    publish(finishedStage);
    takeAlongside();

    return;
  }

  takeAlongside();

  for (uint32_t last = 0u;;) {
    const uint32_t value = awaitStage(last);

    if (value == finishedStage) {
      return;
    }

    last = value;
    claim(value - 1u, worker);
  }
}

void StageRunner::takeAlongside() {
  uint32_t expected = 0u;

  if (m_alongside != nullptr
      && m_alongsideClaimed.compare_exchange_strong(expected, 1u, std::memory_order_acq_rel)) {
    const AlongsideScope scope;

    (*m_alongside)();
  }
}

void StageRunner::claim(uint32_t stage, uint32_t worker) {
  const uint32_t blocks = m_blockCounts[stage];

  if (m_claiming == Claiming::Shared) {
    uint32_t done = 0u;

    for (;;) {
      const uint32_t block = m_next[stage].fetch_add(1u, std::memory_order_acq_rel);

      if (block >= blocks) {
        break;
      }

      m_work(stage, block, worker);
      ++done;
    }

    countDone(stage, done);

    return;
  }

  std::atomic<uint32_t>* flags = m_flags.get() + m_flagStart[stage];
  const uint32_t start = static_cast<uint32_t>(
    static_cast<uint64_t>(worker) * blocks / std::max(m_workers.count, 1u)
  );
  const auto take = [&](uint32_t block) {
    uint32_t expected = 0u;

    return flags[block].compare_exchange_strong(expected, 1u, std::memory_order_acq_rel);
  };
  uint32_t done = 0u;

  for (uint32_t block = start; block < blocks && take(block); ++block) {
    m_work(stage, block, worker);
    ++done;
  }

  for (uint32_t block = start; block-- != 0u && take(block);) {
    m_work(stage, block, worker);
    ++done;
  }

  countDone(stage, done);
}

}
