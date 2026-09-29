#include "win32windowsystem.ih"

namespace talus::capi {

Win32WindowSystem::Win32WindowSystem(void* windowHandle)
    : m_windowHandle(windowHandle) {
  if (m_windowHandle == nullptr) {
    throw std::runtime_error("A session needs a window handle");
  }

  if (IsWindow(static_cast<HWND>(m_windowHandle)) == FALSE) {
    throw std::runtime_error("The window handle does not name a live window");
  }
}

std::vector<const char*> Win32WindowSystem::instanceExtensions() const {
  return std::vector<const char*>{
    VK_KHR_SURFACE_EXTENSION_NAME,
    VK_KHR_WIN32_SURFACE_EXTENSION_NAME
  };
}

VkSurfaceKHR Win32WindowSystem::createSurface(VkInstance instance) const {
  VkWin32SurfaceCreateInfoKHR createInfo{};
  createInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
  createInfo.hwnd = static_cast<HWND>(m_windowHandle);
  createInfo.hinstance = reinterpret_cast<HINSTANCE>(
    GetWindowLongPtrW(createInfo.hwnd, GWLP_HINSTANCE)
  );

  VkSurfaceKHR surface = VK_NULL_HANDLE;

  const VkResult result =
    vkCreateWin32SurfaceKHR(instance, &createInfo, nullptr, &surface);

  if (result != VK_SUCCESS) {
    throw std::runtime_error(
      std::string("vkCreateWin32SurfaceKHR failed: ") + string_VkResult(result)
    );
  }

  return surface;
}

VkExtent2D Win32WindowSystem::framebufferExtent() const {
  RECT client{};

  if (GetClientRect(static_cast<HWND>(m_windowHandle), &client) == FALSE) {
    return VkExtent2D{0, 0};
  }

  return VkExtent2D{
    static_cast<uint32_t>(client.right - client.left),
    static_cast<uint32_t>(client.bottom - client.top)
  };
}

void Win32WindowSystem::waitForNonZeroExtent() const {
  const VkExtent2D extent = framebufferExtent();

  if (extent.width == 0 || extent.height == 0) {
    throw std::runtime_error(
      "The host asked for a frame while the viewport had no area"
    );
  }
}

}
