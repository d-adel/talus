#include "rendererstorage.ih"

namespace talus::renderer {

RendererStorage::RendererStorage(
  const talus::gpu::Device& device,
  std::span<const talus::gpu::Texture> textures,
  std::span<const talus::gpu::Sampler> samplers,
  std::span<const MaterialRecord> materials
)
    : m_uploader(device),
      m_frameGlobals(device),
      m_resourceTable(device, textures, samplers, materials),
      m_recordTable(device),
      m_sdfBaker(device, m_recordTable),
      m_volumePool(device),
      m_residency(m_volumePool, m_recordTable, m_uploader),
      m_framePacker(m_volumePool, m_residency, m_recordTable) {}

talus::gpu::Uploader& RendererStorage::uploader() noexcept {
  return m_uploader;
}

FrameGlobals& RendererStorage::frameGlobals() noexcept {
  return m_frameGlobals;
}

const FrameGlobals& RendererStorage::frameGlobals() const noexcept {
  return m_frameGlobals;
}

ResourceTable& RendererStorage::resourceTable() noexcept {
  return m_resourceTable;
}

SdfRecordTable& RendererStorage::recordTable() noexcept {
  return m_recordTable;
}

const SdfRecordTable& RendererStorage::recordTable() const noexcept {
  return m_recordTable;
}

const SdfBaker& RendererStorage::sdfBaker() const noexcept {
  return m_sdfBaker;
}

VolumePool& RendererStorage::volumePool() noexcept {
  return m_volumePool;
}

const VolumePool& RendererStorage::volumePool() const noexcept {
  return m_volumePool;
}

SdfResidency& RendererStorage::residency() noexcept {
  return m_residency;
}

FramePacker& RendererStorage::framePacker() noexcept {
  return m_framePacker;
}

}
