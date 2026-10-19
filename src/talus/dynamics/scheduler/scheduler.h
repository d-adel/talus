#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

namespace talus::dynamics {

using TaskCallback = void(void* taskContext);
using EnqueueTaskCallback = void*(
  TaskCallback* task,
  void* taskContext,
  void* userContext,
  const char* taskName
);
using FinishTaskCallback = void(void* userTask, void* userContext);

inline constexpr uint32_t maxWorkers = 32u;
inline constexpr uint32_t maxTasks = 256u;

class Scheduler {
public:
  explicit Scheduler(uint32_t workerCount);
  ~Scheduler();

  Scheduler(const Scheduler&) = delete;
  Scheduler& operator=(const Scheduler&) = delete;

  [[nodiscard]] uint32_t workerCount() const;
  void* enqueue(TaskCallback* task, void* taskContext);
  void finish(void* userTask);
  void reset();

private:
  struct Task {
    TaskCallback* callback = nullptr;
    void* context = nullptr;
    std::atomic<int> status{0};
  };

  bool executeOne();
  void post();
  [[nodiscard]] bool awaitWork();
  void serve();

  uint32_t m_workerCount = 1u;
  std::array<Task, maxTasks> m_tasks;
  std::atomic<uint32_t> m_next{0u};
  std::mutex m_mutex;
  std::condition_variable m_wake;
  std::atomic<uint32_t> m_wakeups{0u};
  std::atomic<uint32_t> m_finishers{0u};
  bool m_shutdown = false;
  std::vector<std::thread> m_threads;
};

struct Workers {
  uint32_t count = 1u;
  EnqueueTaskCallback* hostEnqueue = nullptr;
  FinishTaskCallback* hostFinish = nullptr;
  void* hostContext = nullptr;
  Scheduler* scheduler = nullptr;

  void* enqueue(TaskCallback* task, void* taskContext, const char* name) const;
  void finish(void* userTask) const;
  void finishRun() const;
};

[[nodiscard]] Workers resolveWorkers(
  uint32_t workerCount,
  EnqueueTaskCallback* enqueueTask,
  FinishTaskCallback* finishTask,
  void* userTaskContext,
  std::shared_ptr<Scheduler>& scheduler
);

void parallelFor(
  const Workers& workers,
  uint32_t itemCount,
  uint32_t minRange,
  const std::function<void(uint32_t begin, uint32_t end, uint32_t worker)>& callback
);


enum class Claiming : uint8_t { Shared, Ranges };

class StageRunner {
public:
  StageRunner() = default;
  StageRunner(
    const Workers& workers,
    std::vector<uint32_t> blockCounts,
    std::function<void(uint32_t stage, uint32_t block, uint32_t worker)> work
  );

  void reset(
    const Workers& workers,
    std::span<const uint32_t> blockCounts,
    std::function<void(uint32_t stage, uint32_t block, uint32_t worker)> work,
    Claiming claiming = Claiming::Shared
  );
  void run(const std::function<void()>& orchestrate, const std::function<void()>* alongside = nullptr);
  void execute(uint32_t stage);

private:
  struct WorkerTask {
    StageRunner* runner = nullptr;
    uint32_t index = 0u;
  };

  static void serveTask(void* context);
  void serve(uint32_t worker);
  void claim(uint32_t stage, uint32_t worker);
  void takeAlongside();
  void publish(uint32_t value);
  [[nodiscard]] uint32_t awaitStage(uint32_t last);
  void awaitDone(uint32_t stage, uint32_t blocks);
  void countDone(uint32_t stage, uint32_t done);

  Workers m_workers;
  std::vector<uint32_t> m_blockCounts;
  std::vector<uint32_t> m_flagStart;
  std::function<void(uint32_t, uint32_t, uint32_t)> m_work;
  std::unique_ptr<std::atomic<uint32_t>[]> m_flags;
  std::unique_ptr<std::atomic<uint32_t>[]> m_next;
  std::unique_ptr<std::atomic<uint32_t>[]> m_done;
  Claiming m_claiming = Claiming::Shared;
  size_t m_flagCapacity = 0u;
  size_t m_stageCapacity = 0u;
  std::atomic<uint32_t> m_current{0u};
  std::atomic<uint32_t> m_claimed{0u};
  std::atomic<uint32_t> m_alongsideClaimed{0u};
  std::atomic<uint32_t> m_sleepers{0u};
  std::atomic<uint32_t> m_waiting{0u};
  const std::function<void()>* m_orchestrate = nullptr;
  const std::function<void()>* m_alongside = nullptr;
  bool m_parallel = false;
};

}
