# register experiment resources
list(REMOVE_DUPLICATES _EXPERIMENTS_PACKAGE_RESOURCE_INDICES)
foreach(resource_index ${_EXPERIMENTS_PACKAGE_RESOURCE_INDICES})
  ament_index_register_resource(
    ${resource_index} CONTENT "${_EXPERIMENTS_${resource_index}__EXP_LISTS}")
endforeach()