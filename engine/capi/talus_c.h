#pragma once

#include <stdint.h>

#if defined(_WIN32)
  #define TALUS_C_API __declspec(dllexport)
#else
  #define TALUS_C_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define TALUS_C_VERSION 4u

TALUS_C_API uint32_t talus_c_version(void);

typedef struct TalusSession TalusSession;

TALUS_C_API TalusSession* talus_session_create(
  void* windowHandle,
  const char* projectRoot,
  const char* world
);

TALUS_C_API void talus_session_destroy(TalusSession* session);

TALUS_C_API int32_t talus_session_resize(TalusSession* session);

TALUS_C_API int32_t
talus_session_retarget(TalusSession* session, void* windowHandle);

TALUS_C_API int32_t talus_session_render(TalusSession* session);

TALUS_C_API int32_t talus_session_extent(
  const TalusSession* session,
  uint32_t* width,
  uint32_t* height
);

TALUS_C_API int32_t talus_session_entities(
  const TalusSession* session,
  uint32_t* entities,
  uint32_t capacity,
  uint32_t* count
);

TALUS_C_API int32_t talus_session_entity_components(
  const TalusSession* session,
  uint32_t entity,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

#define TALUS_FIELD_OPTIONAL 1u
#define TALUS_COMPONENT_WRITABLE 1u
#define TALUS_COMPONENT_ASSET_BACKED 2u

typedef struct TalusComponentSchema {
  uint32_t size;
  uint32_t fieldCount;
  uint32_t byteSize;
  uint32_t flags;
  uint32_t nameOffset;
  uint32_t defaultsOffset;
} TalusComponentSchema;

typedef struct TalusFieldSchema {
  uint32_t kind;
  uint32_t offset;
  uint32_t size;
  uint32_t flags;
  uint32_t nameOffset;
} TalusFieldSchema;

TALUS_C_API int32_t talus_session_component_schema(
  const TalusSession* session,
  const char* component,
  uint8_t* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_session_component_bytes(
  const TalusSession* session,
  uint32_t entity,
  const char* component,
  uint8_t* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_session_set_field(
  TalusSession* session,
  uint32_t entity,
  const char* component,
  uint32_t field,
  uint32_t kind,
  const uint8_t* bytes,
  uint32_t size
);

TALUS_C_API int32_t talus_session_component_names(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_session_add_component(
  TalusSession* session,
  uint32_t entity,
  const char* component
);
TALUS_C_API int32_t talus_session_remove_component(
  TalusSession* session,
  uint32_t entity,
  const char* component
);
TALUS_C_API int32_t talus_session_can_add_component(
  const TalusSession* session,
  uint32_t entity,
  const char* component
);
TALUS_C_API int32_t talus_session_collections(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);
TALUS_C_API int32_t talus_session_collection_members(
  const TalusSession* session,
  const char* collection,
  uint32_t* entities,
  uint32_t capacity,
  uint32_t* count
);
TALUS_C_API int32_t talus_session_collect(
  TalusSession* session,
  uint32_t entity,
  const char* collection
);
TALUS_C_API int32_t talus_session_uncollect(
  TalusSession* session,
  uint32_t entity,
  const char* collection
);
TALUS_C_API int32_t talus_session_drop_collection(
  TalusSession* session,
  const char* collection
);
TALUS_C_API const void* talus_api(void);

typedef void (*TalusLogSink)(const char* message);

TALUS_C_API void talus_set_log_sink(TalusLogSink sink);

TALUS_C_API void* talus_session_schedule(TalusSession* session);

TALUS_C_API void* talus_session_world(TalusSession* session);

typedef struct TalusProject TalusProject;

TALUS_C_API TalusProject* talus_project_open(const char* root);

TALUS_C_API TalusProject* talus_project_create(
  const char* root,
  const char* name
);

TALUS_C_API void talus_project_close(TalusProject* project);

TALUS_C_API int32_t talus_project_root(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_project_worlds(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_project_assets(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);
TALUS_C_API int32_t talus_project_filters(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API uint64_t talus_project_import_asset(
  TalusProject* project,
  const char* file,
  const char* type
);

TALUS_C_API int32_t talus_project_editor_state_get(
  const TalusProject* project,
  int32_t* autoSaveBeforeRun
);

TALUS_C_API int32_t talus_project_editor_state_set(
  TalusProject* project,
  int32_t autoSaveBeforeRun
);

TALUS_C_API void talus_session_set_key(
  TalusSession* session,
  uint32_t key,
  int32_t down
);

TALUS_C_API void talus_session_set_mouse_button(
  TalusSession* session,
  uint32_t button,
  int32_t down
);

TALUS_C_API void talus_session_set_cursor(
  TalusSession* session,
  float x,
  float y
);

TALUS_C_API uint32_t talus_session_main_camera(const TalusSession* session);

TALUS_C_API int32_t
talus_session_set_main_camera(TalusSession* session, uint32_t entity);

TALUS_C_API uint32_t talus_session_create_light(TalusSession* session);

TALUS_C_API uint32_t talus_session_create_environment(TalusSession* session);

TALUS_C_API uint32_t talus_session_create_camera(
  TalusSession* session,
  const float* position,
  const float* target,
  float verticalFov
);

TALUS_C_API uint32_t talus_session_create_sdf(
  TalusSession* session,
  const float* position,
  const float* rotation,
  const uint32_t* resolution,
  float margin
);

TALUS_C_API int32_t talus_session_add_primitive(
  TalusSession* session,
  uint32_t entity,
  uint64_t material,
  const void* edit
);

TALUS_C_API int32_t
talus_session_set_static(TalusSession* session, uint32_t entity);

TALUS_C_API int32_t
talus_session_set_deformable(TalusSession* session, uint32_t entity);

TALUS_C_API int32_t
talus_session_destroy_entity(TalusSession* session, uint32_t entity);

TALUS_C_API int32_t
talus_session_record(TalusSession* session, const char* label);

TALUS_C_API int32_t talus_session_drain_changes(
  TalusSession* session,
  uint32_t* touched,
  uint32_t capacity,
  uint32_t* count,
  int32_t* wholesale
);

TALUS_C_API int32_t talus_session_commit(TalusSession* session);

TALUS_C_API int32_t talus_session_discard(TalusSession* session);

TALUS_C_API int32_t talus_session_undo(TalusSession* session);

TALUS_C_API int32_t talus_session_redo(TalusSession* session);

TALUS_C_API int32_t talus_session_can_undo(const TalusSession* session);

TALUS_C_API int32_t talus_session_can_redo(const TalusSession* session);

TALUS_C_API int32_t talus_session_undo_label(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_session_redo_label(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_session_save(TalusSession* session);

TALUS_C_API int32_t talus_session_play(TalusSession* session);

TALUS_C_API int32_t talus_session_stop(TalusSession* session);

TALUS_C_API int32_t talus_session_playing(const TalusSession* session);

TALUS_C_API int32_t
talus_session_set_fixed_step(TalusSession* session, float seconds);

TALUS_C_API float talus_session_fixed_step(const TalusSession* session);

TALUS_C_API int32_t talus_session_pause(TalusSession* session, int32_t paused);

TALUS_C_API int32_t talus_session_paused(const TalusSession* session);

TALUS_C_API int32_t talus_session_step(TalusSession* session);

TALUS_C_API uint32_t talus_session_material_count(const TalusSession* session);

TALUS_C_API uint64_t
talus_session_material_at(const TalusSession* session, uint32_t index);

TALUS_C_API uint64_t talus_session_add_material(
  TalusSession* session,
  const float* baseColor,
  float metallic,
  float roughness
);

TALUS_C_API int32_t talus_session_material_physics(
  const TalusSession* session,
  uint64_t material,
  float* four
);

TALUS_C_API int32_t talus_session_set_material_physics(
  TalusSession* session,
  uint64_t material,
  const float* four
);

#define TALUS_NO_VIEWPORT 0xFFFFFFFFu

typedef struct TalusViewport {
  int32_t gridVisible;
  float supersample;
  int32_t drawMode;
  uint32_t fieldRepresentation;
} TalusViewport;

TALUS_C_API uint32_t talus_session_main_viewport(const TalusSession* session);

TALUS_C_API int32_t talus_viewport_configure(
  TalusSession* session,
  uint32_t viewport,
  const TalusViewport* config
);

TALUS_C_API int32_t talus_viewport_get(
  const TalusSession* session,
  uint32_t viewport,
  TalusViewport* config
);

TALUS_C_API uint32_t
talus_viewport_pick(TalusSession* session, uint32_t viewport, float x, float y);

TALUS_C_API int32_t talus_viewport_orbit(
  TalusSession* session,
  uint32_t viewport,
  float yaw,
  float pitch
);

TALUS_C_API int32_t
talus_viewport_pan(TalusSession* session, uint32_t viewport, float x, float y);

TALUS_C_API int32_t
talus_viewport_zoom(TalusSession* session, uint32_t viewport, float delta);

TALUS_C_API int32_t
talus_viewport_focus(TalusSession* session, uint32_t viewport, uint32_t entity);

TALUS_C_API int32_t
talus_viewport_align_to_scene_camera(TalusSession* session, uint32_t viewport);

#define TALUS_NO_AXIS 3u

TALUS_C_API int32_t talus_viewport_handles(
  TalusSession* session,
  uint32_t viewport,
  uint32_t entity
);

TALUS_C_API uint32_t talus_viewport_handle_at(
  TalusSession* session,
  uint32_t viewport,
  float x,
  float y
);

TALUS_C_API int32_t talus_viewport_highlight_handle(
  TalusSession* session,
  uint32_t viewport,
  uint32_t axis
);

TALUS_C_API int32_t talus_viewport_grab_handle(
  TalusSession* session,
  uint32_t viewport,
  uint32_t axis,
  float x,
  float y
);

TALUS_C_API int32_t talus_viewport_drag_handle(
  TalusSession* session,
  uint32_t viewport,
  float x,
  float y
);

TALUS_C_API int32_t
talus_viewport_release_handle(TalusSession* session, uint32_t viewport);

TALUS_C_API int32_t
talus_session_open_world(TalusSession* session, const char* name);

TALUS_C_API int32_t
talus_session_new_world(TalusSession* session, const char* name);

TALUS_C_API int32_t
talus_session_save_as(TalusSession* session, const char* name);

TALUS_C_API int32_t talus_session_package(
  TalusSession* session,
  const char* playerExecutable,
  const char* output,
  const char* worlds,
  char* executable,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_session_current_world(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
);

TALUS_C_API int32_t talus_session_show_debug(
  TalusSession* session,
  const char* category,
  int32_t shown
);

TALUS_C_API const char* talus_last_error(void);

#ifdef __cplusplus
}
#endif
