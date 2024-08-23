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

latest_tag = run_command('git', 'describe', '--tags', '--abbrev=0').stdout().strip()
if latest_tag == ''
  latest_tag = 'NOTAG'
endif
add_project_arguments('-DVERSION_STRING="' + latest_tag + '"', language: 'cpp')

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
