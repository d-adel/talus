#include "talus_c.ih"

#include <talus/objectmodel/worldquery/worldquery.h>

namespace {

thread_local std::string g_lastError;

void clearError() {
  g_lastError.clear();
}

void recordError(const char* what) {
  g_lastError = what;
}

template<typename Fn>
int32_t guard(Fn&& body) {
  clearError();

  try {
    body();

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure across the boundary");
  }

  return 1;
}

talus::capi::Session* asSession(TalusSession* session) {
  return reinterpret_cast<talus::capi::Session*>(session);
}

const talus::capi::Session* asSession(const TalusSession* session) {
  return reinterpret_cast<const talus::capi::Session*>(session);
}

talus::serialize::Project* asProject(TalusProject* project) {
  return reinterpret_cast<talus::serialize::Project*>(project);
}

const talus::serialize::Project* asProject(const TalusProject* project) {
  return reinterpret_cast<const talus::serialize::Project*>(project);
}

TalusProject* asProject(talus::serialize::Project* project) {
  return reinterpret_cast<TalusProject*>(project);
}

talus::capi::SceneView& requireView(TalusSession* session, uint32_t viewport) {
  if (session == nullptr) {
    throw std::runtime_error("A viewport call needs a session");
  }

  talus::capi::SceneView* found = asSession(session)->view(viewport);

  if (found == nullptr) {
    throw std::runtime_error("The session keeps no such viewport");
  }

  return *found;
}

std::vector<std::string> splitLines(const char* text) {
  std::vector<std::string> lines;

  std::string current;

  for (const char* at = text; *at != 0; ++at) {
    if (*at == 0x0a) {
      if (!current.empty()) {
        lines.push_back(current);
      }

      current.clear();

      continue;
    }

    current.push_back(*at);
  }

  if (!current.empty()) {
    lines.push_back(current);
  }

  return lines;
}

void fill(
  const std::string& text,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  *length = static_cast<uint32_t>(text.size());

  if (capacity == 0) {
    return;
  }

  const auto written = std::min<size_t>(capacity - 1, text.size());

  std::memcpy(buffer, text.data(), written);

  buffer[written] = '\0';
}

void fillBytes(
  std::span<const std::byte> bytes,
  uint8_t* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  *length = static_cast<uint32_t>(bytes.size());

  if (capacity == 0 || buffer == nullptr) {
    return;
  }

  std::memcpy(buffer, bytes.data(), std::min<size_t>(capacity, bytes.size()));
}

static_assert(sizeof(TalusComponentSchema) == 24u);
static_assert(sizeof(TalusFieldSchema) == 20u);
static_assert(TALUS_FIELD_OPTIONAL == talus::objectmodel::FIELD_OPTIONAL);
static_assert(TALUS_COMPONENT_WRITABLE == talus::serialize::COMPONENT_WRITABLE);
static_assert(
  TALUS_COMPONENT_ASSET_BACKED == talus::serialize::COMPONENT_ASSET_BACKED
);

std::vector<std::byte> packSchema(
  const talus::serialize::ComponentSchema& schema
) {
  const auto fieldCount = static_cast<uint32_t>(schema.spec.fields.size());

  std::string names = schema.spec.name;
  names.push_back('\0');

  std::vector<uint32_t> fieldNames;

  for (const auto& field : schema.spec.fields) {
    fieldNames.push_back(static_cast<uint32_t>(names.size()));
    names.append(field.name);
    names.push_back('\0');
  }

  const auto headerSize = static_cast<uint32_t>(sizeof(TalusComponentSchema));
  const auto recordSize = static_cast<uint32_t>(sizeof(TalusFieldSchema));
  const uint32_t namesAt = headerSize + fieldCount * recordSize;
  const uint32_t defaultsAt = namesAt + static_cast<uint32_t>(names.size());

  std::vector<std::byte> blob(defaultsAt + schema.defaults.size());

  const TalusComponentSchema header{
    headerSize,
    fieldCount,
    schema.spec.size,
    schema.flags,
    namesAt,
    defaultsAt
  };

  std::memcpy(blob.data(), &header, sizeof(header));

  for (uint32_t index = 0u; index < fieldCount; ++index) {
    const auto& field = schema.spec.fields[index];

    const TalusFieldSchema record{
      static_cast<uint32_t>(field.kind),
      field.offset,
      talus::serialize::fieldByteSize(field.kind, field.flags),
      field.flags,
      namesAt + fieldNames[index]
    };

    std::memcpy(
      blob.data() + headerSize + index * recordSize,
      &record,
      sizeof(record)
    );
  }

  std::memcpy(blob.data() + namesAt, names.data(), names.size());
  std::memcpy(
    blob.data() + defaultsAt,
    schema.defaults.data(),
    schema.defaults.size()
  );

  return blob;
}

}

extern "C" {

TalusSession* talus_session_create(
  void* windowHandle,
  const char* projectRoot,
  const char* world
) {
  clearError();

  try {
    auto windowSystem =
      std::make_unique<talus::capi::Win32WindowSystem>(windowHandle);

    auto session = std::make_unique<talus::capi::Session>(
      std::move(windowSystem),
      projectRoot == nullptr ? "projects/demo" : projectRoot,
      world == nullptr ? "main" : world
    );

    return reinterpret_cast<TalusSession*>(session.release());
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while creating the session");
  }

  return nullptr;
}

void talus_session_destroy(TalusSession* session) {
  clearError();

  try {
    delete asSession(session);
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while destroying the session");
  }
}

int32_t talus_session_resize(TalusSession* session) {
  clearError();

  if (session == nullptr) {
    recordError("talus_session_resize was given no session");

    return 1;
  }

  try {
    asSession(session)->resize();

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while resizing the session");
  }

  return 1;
}

int32_t talus_session_retarget(TalusSession* session, void* windowHandle) {
  clearError();

  if (session == nullptr) {
    recordError("talus_session_retarget was given no session");

    return 1;
  }

  try {
    asSession(session)->retarget(
      std::make_unique<talus::capi::Win32WindowSystem>(windowHandle)
    );

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while retargeting the session");
  }

  return 1;
}

int32_t talus_session_render(TalusSession* session) {
  clearError();

  if (session == nullptr) {
    recordError("talus_session_render was given no session");

    return 1;
  }

  try {
    asSession(session)->render();

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while rendering the session");
  }

  return 1;
}

int32_t talus_session_extent(
  const TalusSession* session,
  uint32_t* width,
  uint32_t* height
) {
  clearError();

  if (session == nullptr || width == nullptr || height == nullptr) {
    recordError("talus_session_extent was given a null argument");

    return 1;
  }

  const VkExtent2D extent = asSession(session)->extent();

  *width = extent.width;
  *height = extent.height;

  return 0;
}

int32_t talus_session_entities(
  const TalusSession* session,
  uint32_t* entities,
  uint32_t capacity,
  uint32_t* count
) {
  clearError();

  if (session == nullptr || count == nullptr) {
    recordError("talus_session_entities was given a null argument");

    return 1;
  }

  if (capacity > 0 && entities == nullptr) {
    recordError("talus_session_entities was given no buffer");

    return 1;
  }

  try {
    const auto live =
      talus::objectmodel::liveEntities(asSession(session)->activeWorld());

    *count = static_cast<uint32_t>(live.size());

    const auto written = std::min<size_t>(capacity, live.size());

    for (size_t index = 0; index < written; ++index) {
      entities[index] = static_cast<uint32_t>(live[index]);
    }

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while listing entities");
  }

  return 1;
}

int32_t talus_session_entity_components(
  const TalusSession* session,
  uint32_t entity,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  clearError();

  if (session == nullptr || length == nullptr) {
    recordError("talus_session_entity_components was given a null argument");

    return 1;
  }

  if (capacity > 0 && buffer == nullptr) {
    recordError("talus_session_entity_components was given no buffer");

    return 1;
  }

  try {
    const auto names = talus::objectmodel::componentNames(
      asSession(session)->activeWorld(),
      static_cast<talus::objectmodel::Entity>(entity)
    );

    std::string joined;

    for (const auto& name : names) {
      if (!joined.empty()) {
        joined.push_back('\n');
      }

      joined.append(name);
    }

    fill(joined, buffer, capacity, length);

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while listing components");
  }

  return 1;
}

int32_t talus_session_component_schema(
  const TalusSession* session,
  const char* component,
  uint8_t* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  clearError();

  if (session == nullptr || component == nullptr || length == nullptr) {
    recordError("talus_session_component_schema was given a null argument");

    return 1;
  }

  if (capacity > 0 && buffer == nullptr) {
    recordError("talus_session_component_schema was given no buffer");

    return 1;
  }

  try {
    const auto schema = asSession(session)->componentSchema(component);

    if (!schema) {
      recordError("no component of that name exists");

      return 1;
    }

    fillBytes(packSchema(*schema), buffer, capacity, length);

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while describing a component");
  }

  return 1;
}

int32_t talus_session_component_bytes(
  const TalusSession* session,
  uint32_t entity,
  const char* component,
  uint8_t* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  clearError();

  if (session == nullptr || component == nullptr || length == nullptr) {
    recordError("talus_session_component_bytes was given a null argument");

    return 1;
  }

  if (capacity > 0 && buffer == nullptr) {
    recordError("talus_session_component_bytes was given no buffer");

    return 1;
  }

  try {
    const auto bytes = asSession(session)->componentBytes(
      static_cast<talus::objectmodel::Entity>(entity),
      component
    );

    if (!bytes) {
      recordError("that entity has no such component");

      return 1;
    }

    fillBytes(*bytes, buffer, capacity, length);

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while reading a component");
  }

  return 1;
}

int32_t talus_session_set_field(
  TalusSession* session,
  uint32_t entity,
  const char* component,
  uint32_t field,
  uint32_t kind,
  const uint8_t* bytes,
  uint32_t size
) {
  return guard([&] {
    if (
      session == nullptr || component == nullptr
      || (size > 0 && bytes == nullptr)
    ) {
      throw std::runtime_error(
        "talus_session_set_field was given a null argument"
      );
    }

    if (!talus::objectmodel::isFieldKind(kind)) {
      throw std::runtime_error("unknown field kind");
    }

    const auto status = asSession(session)->setField(
      static_cast<talus::objectmodel::Entity>(entity),
      component,
      field,
      static_cast<talus::objectmodel::FieldKind>(kind),
      std::span<const std::byte>(
        reinterpret_cast<const std::byte*>(bytes),
        size
      )
    );

    if (status != talus::serialize::WriteStatus::Written) {
      throw std::runtime_error(std::string(talus::serialize::describe(status)));
    }
  });
}

int32_t talus_session_component_names(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  clearError();

  if (session == nullptr || length == nullptr) {
    recordError("talus_session_component_names was given a null argument");

    return 1;
  }

  if (capacity > 0 && buffer == nullptr) {
    recordError("talus_session_component_names was given no buffer");

    return 1;
  }

  try {
    std::string joined;

    for (const auto& name : asSession(session)->componentNames()) {
      if (!joined.empty()) {
        joined.push_back('\n');
      }

      joined.append(name);
    }

    fill(joined, buffer, capacity, length);

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  } catch (...) {
    recordError("Unknown failure while listing component names");
  }

  return 1;
}

int32_t talus_session_add_component(
  TalusSession* session,
  uint32_t entity,
  const char* component
) {
  return guard([&] {
    if (session == nullptr || component == nullptr) {
      throw std::runtime_error(
        "talus_session_add_component was given a null argument"
      );
    }

    if (!asSession(session)->addComponent(
          static_cast<talus::objectmodel::Entity>(entity),
          component
        )) {
      throw std::runtime_error(
        "no such component, a dead entity, a component already carried, or "
        "a requirement the entity lacks"
      );
    }
  });
}

int32_t talus_session_collections(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  clearError();

  if (session == nullptr || length == nullptr) {
    recordError("talus_session_collections was given a null argument");

    return 1;
  }

  if (capacity > 0 && buffer == nullptr) {
    recordError("talus_session_collections was given no buffer");

    return 1;
  }

  try {
    std::string joined;

    for (const auto& held : asSession(session)->activeWorld().collections()) {
      if (!joined.empty()) {
        joined.push_back('\n');
      }

      joined += held.name;
    }

    *length = static_cast<uint32_t>(joined.size());

    if (capacity > joined.size()) {
      std::memcpy(buffer, joined.data(), joined.size());
      buffer[joined.size()] = '\0';
    }

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());

    return 1;
  }
}

int32_t talus_session_collection_members(
  const TalusSession* session,
  const char* collection,
  uint32_t* entities,
  uint32_t capacity,
  uint32_t* count
) {
  clearError();

  if (session == nullptr || collection == nullptr || count == nullptr) {
    recordError("talus_session_collection_members was given a null argument");

    return 1;
  }

  if (capacity > 0 && entities == nullptr) {
    recordError("talus_session_collection_members was given no buffer");

    return 1;
  }

  try {
    const auto* held = asSession(session)->activeWorld().collection(collection);

    if (held == nullptr) {
      recordError("no such collection");

      return 1;
    }

    *count = static_cast<uint32_t>(held->members.size());

    const auto written = std::min<size_t>(capacity, held->members.size());

    for (size_t index = 0; index < written; ++index) {
      entities[index] = static_cast<uint32_t>(held->members[index]);
    }

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());

    return 1;
  }
}

int32_t talus_session_collect(
  TalusSession* session,
  uint32_t entity,
  const char* collection
) {
  return guard([&] {
    if (session == nullptr || collection == nullptr) {
      throw std::runtime_error(
        "talus_session_collect was given a null argument"
      );
    }

    if (!asSession(session)->activeWorld().collect(
          collection,
          static_cast<talus::objectmodel::Entity>(entity)
        )) {
      throw std::runtime_error(
        "a dead entity, an empty name, or already a member"
      );
    }
  });
}

int32_t talus_session_uncollect(
  TalusSession* session,
  uint32_t entity,
  const char* collection
) {
  return guard([&] {
    if (session == nullptr || collection == nullptr) {
      throw std::runtime_error(
        "talus_session_uncollect was given a null argument"
      );
    }

    if (!asSession(session)->activeWorld().uncollect(
          collection,
          static_cast<talus::objectmodel::Entity>(entity)
        )) {
      throw std::runtime_error("not a member of that collection");
    }
  });
}

int32_t talus_session_drop_collection(
  TalusSession* session,
  const char* collection
) {
  return guard([&] {
    if (session == nullptr || collection == nullptr) {
      throw std::runtime_error(
        "talus_session_drop_collection was given a null argument"
      );
    }

    if (!asSession(session)->activeWorld().dropCollection(collection)) {
      throw std::runtime_error("no such collection");
    }
  });
}

int32_t talus_session_can_add_component(
  const TalusSession* session,
  uint32_t entity,
  const char* component
) {
  clearError();

  if (session == nullptr || component == nullptr) {
    return 0;
  }

  try {
    return asSession(session)->canAddComponent(
             static_cast<talus::objectmodel::Entity>(entity),
             component
           )
             ? 1
             : 0;
  } catch (const std::exception&) {
    return 0;
  }
}

int32_t talus_session_remove_component(
  TalusSession* session,
  uint32_t entity,
  const char* component
) {
  return guard([&] {
    if (session == nullptr || component == nullptr) {
      throw std::runtime_error(
        "talus_session_remove_component was given a null argument"
      );
    }

    const auto subject = static_cast<talus::objectmodel::Entity>(entity);

    if (!asSession(session)->removeComponent(subject, component)) {
      const std::string blocker =
        asSession(session)->removalBlocker(subject, component);

      throw std::runtime_error(
        blocker.empty()
          ? std::string(
              "no such component on that entity, or it cannot be removed"
            )
          : std::string(component) + " is required by " + blocker
      );
    }
  });
}

uint32_t talus_c_version(void) {
  return TALUS_C_VERSION;
}

int32_t talus_session_show_debug(
  TalusSession* session,
  const char* category,
  int32_t shown
) {
  if (session == nullptr || category == nullptr) {
    return 0;
  }

  asSession(session)->showDebug(category, shown != 0);

  return 1;
}

void talus_set_log_sink(TalusLogSink sink) {
  talus::script::setLogSink(sink);
}

const void* talus_api(void) {
  return &talus::script::api();
}

void* talus_session_schedule(TalusSession* session) {
  if (session == nullptr) {
    return nullptr;
  }

  return &asSession(session)->schedule();
}

void* talus_session_world(TalusSession* session) {
  if (session == nullptr) {
    return nullptr;
  }

  return &asSession(session)->world();
}

TalusProject* talus_project_open(const char* root) {
  clearError();

  if (root == nullptr) {
    recordError("Opening a project needs a root");

    return nullptr;
  }

  try {
    std::optional<talus::serialize::Project> opened =
      talus::serialize::Project::open(root);

    if (!opened.has_value()) {
      recordError("No project was found at that root");

      return nullptr;
    }

    return asProject(new talus::serialize::Project(std::move(opened).value()));
  } catch (const std::exception& failure) {
    recordError(failure.what());
  }

  return nullptr;
}

TalusProject* talus_project_create(const char* root, const char* name) {
  clearError();

  if (root == nullptr || name == nullptr) {
    recordError("A project needs a root and a name");

    return nullptr;
  }

  try {
    std::optional<talus::serialize::Project> made =
      talus::serialize::Project::create(root, name);

    if (!made.has_value()) {
      throw std::runtime_error("The project could not be created");
    }

    talus::serialize::StoredMaterial material{};
    material.baseColor = glm::vec4(0.62f, 0.60f, 0.58f, 1.0f);
    material.roughness = 0.85f;

    const std::filesystem::path staged =
      std::filesystem::temp_directory_path() / "default.mat.json";

    if (!talus::serialize::writeText(
          staged,
          talus::serialize::writeMaterial(material)
        )) {
      throw std::runtime_error("The default material could not be written");
    }

    const talus::core::AssetId minted = made->importAsset(staged, "material");

    std::error_code ignored;
    std::filesystem::remove(staged, ignored);

    if (minted == talus::core::INVALID_ASSET) {
      throw std::runtime_error("The default material could not be registered");
    }

    if (!made->save()) {
      throw std::runtime_error("The project manifest could not be saved");
    }

    if (!made->saveWorld("main", talus::serialize::StoredWorld{})) {
      throw std::runtime_error("The project has no world");
    }

    return asProject(new talus::serialize::Project(std::move(made).value()));
  } catch (const std::exception& failure) {
    recordError(failure.what());
  }

  return nullptr;
}

void talus_project_close(TalusProject* project) {
  delete asProject(project);
}

int32_t talus_project_root(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  return guard([&] {
    if (project == nullptr || length == nullptr) {
      throw std::runtime_error("Reading a project root needs a project");
    }

    fill(asProject(project)->root().string(), buffer, capacity, length);
  });
}

int32_t talus_project_worlds(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  return guard([&] {
    if (project == nullptr || length == nullptr) {
      throw std::runtime_error("Listing worlds needs a project");
    }

    std::string joined;

    for (const std::string& name : asProject(project)->worldNames()) {
      if (!joined.empty()) {
        joined.push_back('\n');
      }

      joined += name;
    }

    fill(joined, buffer, capacity, length);
  });
}

int32_t talus_project_assets(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  return guard([&] {
    if (project == nullptr || length == nullptr) {
      throw std::runtime_error("Listing assets needs a project");
    }

    std::string joined;

    for (const talus::serialize::AssetEntry& entry :
         asProject(project)->manifest().assets) {
      if (!joined.empty()) {
        joined.push_back('\n');
      }

      joined +=
        std::to_string(entry.id) + '\t' + entry.type + '\t' + entry.path;
    }

    fill(joined, buffer, capacity, length);
  });
}

int32_t talus_project_filters(
  const TalusProject* project,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  return guard([&] {
    if (project == nullptr || length == nullptr) {
      throw std::runtime_error("Listing filters needs a project");
    }

    std::string joined;

    for (const talus::serialize::FilterEntry& entry :
         asProject(project)->manifest().filters) {
      if (!joined.empty()) {
        joined.push_back('\n');
      }

      joined += entry.name;
      joined.push_back('\t');

      for (size_t index = 0; index < entry.components.size(); ++index) {
        if (index != 0) {
          joined.push_back(',');
        }

        joined += entry.components[index];
      }
    }

    fill(joined, buffer, capacity, length);
  });
}

uint64_t talus_project_import_asset(
  TalusProject* project,
  const char* file,
  const char* type
) {
  clearError();

  if (project == nullptr || file == nullptr || type == nullptr) {
    recordError("Importing an asset needs a project, a file and a type");

    return 0u;
  }

  try {
    talus::serialize::Project* held = asProject(project);

    const talus::core::AssetId minted = held->importAsset(file, type);

    if (minted == talus::core::INVALID_ASSET) {
      throw std::runtime_error("The asset could not be imported");
    }

    if (!held->save()) {
      throw std::runtime_error("The project manifest could not be saved");
    }

    return minted;
  } catch (const std::exception& failure) {
    recordError(failure.what());
  }

  return 0u;
}

int32_t talus_project_editor_state_get(
  const TalusProject* project,
  int32_t* autoSaveBeforeRun
) {
  return guard([&] {
    if (project == nullptr || autoSaveBeforeRun == nullptr) {
      throw std::runtime_error("Reading editor state needs a project");
    }

    talus::serialize::EditorState state;

    static_cast<void>(
      talus::serialize::loadEditorState(asProject(project)->root(), state)
    );

    *autoSaveBeforeRun = state.autoSaveBeforeRun ? 1 : 0;
  });
}

int32_t talus_project_editor_state_set(
  TalusProject* project,
  int32_t autoSaveBeforeRun
) {
  return guard([&] {
    if (project == nullptr) {
      throw std::runtime_error("Writing editor state needs a project");
    }

    talus::serialize::EditorState state;

    static_cast<void>(
      talus::serialize::loadEditorState(asProject(project)->root(), state)
    );

    state.autoSaveBeforeRun = autoSaveBeforeRun != 0;

    if (!talus::serialize::saveEditorState(asProject(project)->root(), state)) {
      throw std::runtime_error("Editor state could not be saved");
    }
  });
}

void talus_session_set_key(
  TalusSession* session,
  uint32_t virtualKey,
  int32_t down
) {
  if (session != nullptr) {
    asSession(session)->setKey(
      static_cast<uint32_t>(talus::capi::keyFromWin32(virtualKey)),
      down != 0
    );
  }
}

void talus_session_set_mouse_button(
  TalusSession* session,
  uint32_t button,
  int32_t down
) {
  if (session != nullptr) {
    asSession(session)->setMouseButton(button, down != 0);
  }
}

void talus_session_set_cursor(TalusSession* session, float x, float y) {
  if (session != nullptr) {
    asSession(session)->setCursor(x, y);
  }
}

uint32_t talus_session_main_camera(const TalusSession* session) {
  if (session == nullptr) {
    return UINT32_MAX;
  }

  const talus::objectmodel::Entity eye =
    talus::objectmodel::mainCamera(asSession(session)->activeWorld());

  return eye == talus::objectmodel::INVALID_ENTITY
           ? UINT32_MAX
           : static_cast<uint32_t>(entt::to_integral(eye));
}

int32_t talus_session_set_main_camera(TalusSession* session, uint32_t entity) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    if (!asSession(session)->world().setMainCamera(
          static_cast<talus::objectmodel::Entity>(entity)
        )) {
      throw std::runtime_error("That entity has no camera to make main");
    }
  });
}

uint32_t talus_session_create_light(TalusSession* session) {
  if (session == nullptr) {
    return UINT32_MAX;
  }

  return static_cast<uint32_t>(
    entt::to_integral(asSession(session)->createLight())
  );
}

uint32_t talus_session_create_environment(TalusSession* session) {
  if (session == nullptr) {
    return UINT32_MAX;
  }

  return static_cast<uint32_t>(
    entt::to_integral(asSession(session)->createEnvironment())
  );
}

uint32_t talus_session_create_camera(
  TalusSession* session,
  const float* position,
  const float* target,
  float verticalFov
) {
  if (session == nullptr || position == nullptr || target == nullptr) {
    return UINT32_MAX;
  }

  auto* live = asSession(session);

  const talus::objectmodel::Entity entity = live->createCamera(
    glm::vec3(position[0], position[1], position[2]),
    glm::vec3(target[0], target[1], target[2]),
    verticalFov
  );

  return static_cast<uint32_t>(entt::to_integral(entity));
}

uint32_t talus_session_create_sdf(
  TalusSession* session,
  const float* position,
  const float* rotation,
  const uint32_t* resolution,
  float margin
) {
  if (
    session == nullptr || position == nullptr || rotation == nullptr
    || resolution == nullptr
  ) {
    return UINT32_MAX;
  }

  auto* live = asSession(session);

  talus::objectmodel::Pose pose{};
  pose.position = glm::vec3(position[0], position[1], position[2]);
  pose.rotation = glm::quat(rotation[3], rotation[0], rotation[1], rotation[2]);

  const talus::objectmodel::Entity entity = live->createSdf(
    pose,
    glm::uvec3(resolution[0], resolution[1], resolution[2]),
    margin
  );

  return static_cast<uint32_t>(entt::to_integral(entity));
}

int32_t talus_session_add_primitive(
  TalusSession* session,
  uint32_t entity,
  uint64_t material,
  const void* edit
) {
  return guard([&] {
    if (session == nullptr || edit == nullptr) {
      throw std::runtime_error("A primitive needs a session and an edit");
    }

    auto* live = asSession(session);

    const auto subject =
      static_cast<talus::objectmodel::Entity>(entt::entity{entity});

    talus::script::SdfEditDescriptor described =
      *static_cast<const talus::script::SdfEditDescriptor*>(edit);

    const uint32_t index = live->materialIndex(material);

    if (index == talus::core::UNBOUND_ASSET) {
      throw std::runtime_error("The material is not in this project");
    }

    described.material = index;

    if (!live->addPrimitive(
          subject,
          talus::script::primitiveFor(live->world(), subject, described)
        )) {
      throw std::runtime_error("The primitive was refused");
    }
  });
}

int32_t talus_session_set_static(TalusSession* session, uint32_t entity) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    asSession(session)->setStatic(
      static_cast<talus::objectmodel::Entity>(entt::entity{entity})
    );
  });
}

int32_t talus_session_set_deformable(TalusSession* session, uint32_t entity) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    asSession(session)->setDeformable(
      static_cast<talus::objectmodel::Entity>(entt::entity{entity})
    );
  });
}

int32_t talus_session_destroy_entity(TalusSession* session, uint32_t entity) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    if (!asSession(session)->destroyEntity(
          static_cast<talus::objectmodel::Entity>(entt::entity{entity})
        )) {
      throw std::runtime_error("The entity could not be destroyed");
    }
  });
}

int32_t talus_session_record(TalusSession* session, const char* label) {
  return guard([&] {
    if (session == nullptr || label == nullptr) {
      throw std::runtime_error("talus_session_record was given no session");
    }

    if (!asSession(session)->record(label)) {
      throw std::runtime_error(
        "a transaction is already open, or the session is playing"
      );
    }
  });
}

int32_t talus_session_drain_changes(
  TalusSession* session,
  uint32_t* touched,
  uint32_t capacity,
  uint32_t* count,
  int32_t* wholesale
) {
  return guard([&] {
    if (session == nullptr || count == nullptr || wholesale == nullptr) {
      throw std::runtime_error(
        "talus_session_drain_changes was given a null argument"
      );
    }

    if (capacity > 0 && touched == nullptr) {
      throw std::runtime_error(
        "talus_session_drain_changes was given no buffer"
      );
    }

    bool everything = false;

    *count = asSession(session)->drainChanges(
      std::span<uint32_t>(touched, capacity),
      everything
    );
    *wholesale = everything ? 1 : 0;
  });
}

int32_t talus_session_commit(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    if (!asSession(session)->commit()) {
      throw std::runtime_error("no transaction is open to commit");
    }
  });
}

int32_t talus_session_discard(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    if (!asSession(session)->discard()) {
      throw std::runtime_error("no transaction is open to discard");
    }
  });
}

int32_t talus_session_undo(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    static_cast<void>(asSession(session)->undo());
  });
}

int32_t talus_session_redo(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    static_cast<void>(asSession(session)->redo());
  });
}

int32_t talus_session_can_undo(const TalusSession* session) {
  return session != nullptr && asSession(session)->undoLabel().has_value() ? 1
                                                                           : 0;
}

int32_t talus_session_can_redo(const TalusSession* session) {
  return session != nullptr && asSession(session)->redoLabel().has_value() ? 1
                                                                           : 0;
}

int32_t talus_session_undo_label(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  return guard([&] {
    if (session == nullptr || length == nullptr) {
      throw std::runtime_error("talus_session_undo_label was given no session");
    }

    if (capacity > 0 && buffer == nullptr) {
      throw std::runtime_error("talus_session_undo_label was given no buffer");
    }

    const std::optional<std::string_view> label =
      asSession(session)->undoLabel();

    fill(
      std::string{label.value_or(std::string_view{})},
      buffer,
      capacity,
      length
    );
  });
}

int32_t talus_session_redo_label(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  return guard([&] {
    if (session == nullptr || length == nullptr) {
      throw std::runtime_error("talus_session_redo_label was given no session");
    }

    if (capacity > 0 && buffer == nullptr) {
      throw std::runtime_error("talus_session_redo_label was given no buffer");
    }

    const std::optional<std::string_view> label =
      asSession(session)->redoLabel();

    fill(
      std::string{label.value_or(std::string_view{})},
      buffer,
      capacity,
      length
    );
  });
}

int32_t talus_session_save(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    if (!asSession(session)->save()) {
      throw std::runtime_error("The world could not be saved");
    }
  });
}

int32_t talus_session_play(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    asSession(session)->play();
  });
}

int32_t talus_session_stop(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    asSession(session)->stop();
  });
}

int32_t talus_session_playing(const TalusSession* session) {
  return session != nullptr && asSession(session)->playing() ? 1 : 0;
}

int32_t talus_session_set_fixed_step(TalusSession* session, float seconds) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    asSession(session)->setFixedStep(
      seconds == 0.0f ? std::optional<float>{} : std::optional<float>{seconds}
    );
  });
}

float talus_session_fixed_step(const TalusSession* session) {
  return session == nullptr ? 0.0f
                            : asSession(session)->fixedStep().value_or(0.0f);
}

int32_t talus_session_pause(TalusSession* session, int32_t paused) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    asSession(session)->setPaused(paused != 0);
  });
}

int32_t talus_session_paused(const TalusSession* session) {
  return session != nullptr && asSession(session)->paused() ? 1 : 0;
}

int32_t talus_session_step(TalusSession* session) {
  return guard([&] {
    if (session == nullptr) {
      throw std::runtime_error("No session");
    }

    if (!asSession(session)->playing()) {
      throw std::runtime_error("the session is not playing");
    }

    asSession(session)->stepOnce();
  });
}

uint32_t talus_session_material_count(const TalusSession* session) {
  return session == nullptr
           ? 0u
           : static_cast<uint32_t>(asSession(session)->materialCount());
}

uint64_t talus_session_material_at(
  const TalusSession* session,
  uint32_t index
) {
  return session == nullptr ? 0u : asSession(session)->materialAt(index);
}

uint64_t talus_session_add_material(
  TalusSession* session,
  const float* baseColor,
  float metallic,
  float roughness
) {
  if (session == nullptr || baseColor == nullptr) {
    return 0u;
  }

  auto* live = asSession(session);

  try {
    return live->addMaterial(
      glm::vec4(baseColor[0], baseColor[1], baseColor[2], baseColor[3]),
      metallic,
      roughness
    );
  } catch (const std::exception& failure) {
    g_lastError = failure.what();

    return 0u;
  }
}

int32_t talus_session_material_physics(
  const TalusSession* session,
  uint64_t material,
  float* four
) {
  return guard([&] {
    if (session == nullptr || four == nullptr) {
      throw std::runtime_error(
        "talus_session_material_physics was given a null argument"
      );
    }

    const talus::objectmodel::PhysicalMaterial physical =
      asSession(session)->materialPhysics(material);

    four[0] = physical.friction;
    four[1] = physical.restitution;
    four[2] = physical.density;
    four[3] = physical.strength;
  });
}

int32_t talus_session_set_material_physics(
  TalusSession* session,
  uint64_t material,
  const float* four
) {
  return guard([&] {
    if (session == nullptr || four == nullptr) {
      throw std::runtime_error(
        "talus_session_set_material_physics was given a null argument"
      );
    }

    talus::objectmodel::PhysicalMaterial physical =
      asSession(session)->materialPhysics(material);

    physical.friction = four[0];
    physical.restitution = four[1];
    physical.density = four[2];
    physical.strength = four[3];

    asSession(session)->setMaterialPhysics(material, physical);
  });
}

int32_t talus_viewport_handles(
  TalusSession* session,
  uint32_t viewport,
  uint32_t entity
) {
  return guard([&] {
    requireView(session, viewport)
      .placeHandles(
        entity == UINT32_MAX
          ? talus::objectmodel::INVALID_ENTITY
          : static_cast<talus::objectmodel::Entity>(entt::entity{entity})
      );
  });
}

uint32_t talus_viewport_handle_at(
  TalusSession* session,
  uint32_t viewport,
  float x,
  float y
) {
  if (session == nullptr) {
    return TALUS_NO_AXIS;
  }

  const talus::capi::SceneView* found = asSession(session)->view(viewport);

  return found == nullptr ? TALUS_NO_AXIS : found->handleAxis(x, y);
}

int32_t talus_viewport_highlight_handle(
  TalusSession* session,
  uint32_t viewport,
  uint32_t axis
) {
  return guard([&] { requireView(session, viewport).highlightHandle(axis); });
}

int32_t talus_viewport_grab_handle(
  TalusSession* session,
  uint32_t viewport,
  uint32_t axis,
  float x,
  float y
) {
  return guard([&] {
    if (!requireView(session, viewport).startHandleDrag(axis, x, y)) {
      throw std::runtime_error("The viewport has no handle on that axis");
    }
  });
}

int32_t talus_viewport_drag_handle(
  TalusSession* session,
  uint32_t viewport,
  float x,
  float y
) {
  return guard([&] {
    if (!requireView(session, viewport).moveHandles(x, y)) {
      throw std::runtime_error("The viewport is not dragging a handle");
    }
  });
}

int32_t talus_viewport_release_handle(
  TalusSession* session,
  uint32_t viewport
) {
  return guard([&] { requireView(session, viewport).endHandleDrag(); });
}

uint32_t talus_session_main_viewport(const TalusSession* session) {
  return session == nullptr ? TALUS_NO_VIEWPORT
                            : asSession(session)->mainView().id();
}

int32_t talus_viewport_configure(
  TalusSession* session,
  uint32_t viewport,
  const TalusViewport* config
) {
  return guard([&] {
    if (config == nullptr) {
      throw std::runtime_error("Configuring a viewport needs a config");
    }

    talus::capi::SceneView& found = requireView(session, viewport);

    talus::view::Viewport wanted = found.config();

    wanted.gridVisible = config->gridVisible != 0;
    wanted.supersample = config->supersample;
    wanted.mode = static_cast<talus::view::DrawMode>(config->drawMode);
    wanted.fieldRepresentation = config->fieldRepresentation;

    found.configure(wanted);
  });
}

int32_t talus_viewport_get(
  const TalusSession* session,
  uint32_t viewport,
  TalusViewport* config
) {
  return guard([&] {
    if (session == nullptr || config == nullptr) {
      throw std::runtime_error(
        "Reading a viewport needs a session and a config"
      );
    }

    const talus::capi::SceneView* found = asSession(session)->view(viewport);

    if (found == nullptr) {
      throw std::runtime_error("The session keeps no such viewport");
    }

    const talus::view::Viewport& held = found->config();

    config->gridVisible = held.gridVisible ? 1 : 0;
    config->supersample = held.supersample;
    config->drawMode = static_cast<int32_t>(held.mode);
    config->fieldRepresentation = held.fieldRepresentation;
  });
}

uint32_t talus_viewport_pick(
  TalusSession* session,
  uint32_t viewport,
  float x,
  float y
) {
  if (session == nullptr) {
    return UINT32_MAX;
  }

  const talus::capi::SceneView* found = asSession(session)->view(viewport);

  if (found == nullptr) {
    return UINT32_MAX;
  }

  talus::objectmodel::Entity hit = talus::objectmodel::INVALID_ENTITY;

  guard([&] { hit = found->pick(x, y); });

  return hit == talus::objectmodel::INVALID_ENTITY
           ? UINT32_MAX
           : static_cast<uint32_t>(entt::to_integral(hit));
}

int32_t talus_viewport_orbit(
  TalusSession* session,
  uint32_t viewport,
  float yaw,
  float pitch
) {
  return guard([&] { requireView(session, viewport).orbit(yaw, pitch); });
}

int32_t talus_viewport_pan(
  TalusSession* session,
  uint32_t viewport,
  float x,
  float y
) {
  return guard([&] { requireView(session, viewport).pan(x, y); });
}

int32_t talus_viewport_zoom(
  TalusSession* session,
  uint32_t viewport,
  float delta
) {
  return guard([&] { requireView(session, viewport).zoom(delta); });
}

int32_t talus_viewport_focus(
  TalusSession* session,
  uint32_t viewport,
  uint32_t entity
) {
  return guard([&] {
    requireView(session, viewport)
      .frameOn(
        entity == UINT32_MAX
          ? talus::objectmodel::INVALID_ENTITY
          : static_cast<talus::objectmodel::Entity>(entt::entity{entity})
      );
  });
}

int32_t talus_viewport_align_to_scene_camera(
  TalusSession* session,
  uint32_t viewport
) {
  return guard([&] { requireView(session, viewport).alignToSceneCamera(); });
}

int32_t talus_session_open_world(TalusSession* session, const char* name) {
  return guard([&] {
    if (session == nullptr || name == nullptr) {
      throw std::runtime_error("Opening a world needs a session and a name");
    }

    if (!asSession(session)->openWorld(name)) {
      throw std::runtime_error(std::string("No world named ") + name);
    }
  });
}

int32_t talus_session_new_world(TalusSession* session, const char* name) {
  return guard([&] {
    if (session == nullptr || name == nullptr) {
      throw std::runtime_error("A new world needs a session and a name");
    }

    if (!asSession(session)->newWorld(name)) {
      throw std::runtime_error(std::string("Could not create ") + name);
    }
  });
}

int32_t talus_session_save_as(TalusSession* session, const char* name) {
  return guard([&] {
    if (session == nullptr || name == nullptr) {
      throw std::runtime_error("Saving needs a session and a name");
    }

    if (!asSession(session)->saveAs(name)) {
      throw std::runtime_error(std::string("Could not save as ") + name);
    }
  });
}

int32_t talus_session_package(
  TalusSession* session,
  const char* playerExecutable,
  const char* output,
  const char* worlds,
  char* executable,
  uint32_t capacity,
  uint32_t* length
) {
  clearError();

  if (
    session == nullptr || playerExecutable == nullptr || output == nullptr
    || worlds == nullptr || length == nullptr
  ) {
    recordError("talus_session_package was given a null argument");

    return 1;
  }

  try {
    std::string packaged;

    if (!asSession(session)->package(
          playerExecutable,
          output,
          splitLines(worlds),
          packaged
        )) {
      recordError(packaged.c_str());

      return 1;
    }

    fill(packaged, executable, capacity, length);

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());

    return 1;
  }
}

int32_t talus_session_current_world(
  const TalusSession* session,
  char* buffer,
  uint32_t capacity,
  uint32_t* length
) {
  clearError();

  if (session == nullptr || length == nullptr) {
    recordError("talus_session_current_world was given a null argument");

    return 1;
  }

  try {
    fill(asSession(session)->worldName(), buffer, capacity, length);

    return 0;
  } catch (const std::exception& error) {
    recordError(error.what());
  }

  return 1;
}

const char* talus_last_error(void) {
  return g_lastError.c_str();
}
}
