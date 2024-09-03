project('TwistSpline', 'cpp')

maya_dep = dependency('maya')
maya_name_suffix = maya_dep.get_variable('name_suffix')
maya_version = maya_dep.get_variable('maya_version')

source_files = [
  'src/pluginMain.cpp',
  'src/drawOverride.cpp',
  'src/twistSplineData.cpp',
  'src/twistSplineNode.cpp',
  'src/riderConstraint.cpp',
  'src/twistTangentNode.cpp',
  'src/twistMultiTangentNode.cpp',
]

# If a user-built version file exists, then just use that
# Otherwise grab the latest tag from git
fs = import('fs')
if fs.is_file('src/version.h')
  message('Using existing version.h')
else
  git = find_program('git', native: true, required: true)
  version_h = vcs_tag(
    command: [git, 'describe', '--tags', '--match', 'v[0-9]*', '--dirty=+'],
    fallback: 'v0.0.1',
    input: 'src/version.h.in',
    output: 'version.h',
  )
  source_files = source_files + version_h
endif

outlib = shared_library(
  meson.project_name(),
  source_files,
  install: true,
  install_dir : meson.global_source_root() / 'output_Maya' + maya_version,
  include_directories : include_directories(['src']),
  dependencies : maya_dep,
  name_prefix : '',
  name_suffix : maya_name_suffix,
)
