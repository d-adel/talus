#pragma once

#include <talus/gpu/device/device.h>
#include <talus/gpu/sampler/sampler.h>
#include <talus/gpu/texture/texture.h>
#include <talus/gpu/uploader/uploader.h>
#include <talus/renderer/framepacker/framepacker.h>
#include <talus/renderer/frameglobals/frameglobals.h>
#include <talus/renderer/materialrecord/materialrecord.h>
#include <talus/renderer/resourcetable/resourcetable.h>
#include <talus/renderer/sdfbaker/sdfbaker.h>
#include <talus/renderer/sdfrecordtable/sdfrecordtable.h>
#include <talus/renderer/sdfresidency/sdfresidency.h>
#include <talus/renderer/volumepool/volumepool.h>

#include <span>

namespace talus::renderer {

class RendererStorage {
public:
  RendererStorage(
    const talus::gpu::Device& device,
    std::span<const talus::gpu::Texture> textures,
    std::span<const talus::gpu::Sampler> samplers,
    std::span<const MaterialRecord> materials
  );

  RendererStorage(const RendererStorage&) = delete;
  RendererStorage& operator=(const RendererStorage&) = delete;
  RendererStorage(RendererStorage&&) = delete;
  RendererStorage& operator=(RendererStorage&&) = delete;

  ~RendererStorage() = default;

  [[nodiscard]] talus::gpu::Uploader& uploader() noexcept;

  [[nodiscard]] FrameGlobals& frameGlobals() noexcept;

  [[nodiscard]] const FrameGlobals& frameGlobals() const noexcept;

  [[nodiscard]] ResourceTable& resourceTable() noexcept;

  [[nodiscard]] SdfRecordTable& recordTable() noexcept;

  [[nodiscard]] const SdfRecordTable& recordTable() const noexcept;

  [[nodiscard]] const SdfBaker& sdfBaker() const noexcept;

  [[nodiscard]] VolumePool& volumePool() noexcept;

  [[nodiscard]] const VolumePool& volumePool() const noexcept;

  [[nodiscard]] SdfResidency& residency() noexcept;

  [[nodiscard]] FramePacker& framePacker() noexcept;

private:
  talus::gpu::Uploader m_uploader;
  FrameGlobals m_frameGlobals;
  ResourceTable m_resourceTable;
  SdfRecordTable m_recordTable;
  SdfBaker m_sdfBaker;
  VolumePool m_volumePool;
  SdfResidency m_residency;
  FramePacker m_framePacker;
};

}
