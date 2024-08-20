project('maya', 'cpp')

maya_version = get_option('maya_version')

os_name = build_machine.system()

maya_inc_suffix = 'include'
maya_lib_suffix = 'lib'
maya_bin_suffix = 'bin'

maya_compile_args = ['-DREQUIRE_IOSTREAM', '-D_BOOL']
maya_link_args = []

if os_name == 'windows'
  maya_install_base = 'c:/Program Files/Autodesk'
  maya_plugin_ext = 'mll'
  maya_compile_args += ['-DNT_PLUGIN']
  maya_link_args = ['/export:initializePlugin', '/export:uninitializePlugin']
elif os_name == 'darwin'
  maya_install_base = '/Applications/Autodesk'
  maya_inc_suffix = 'devkit/include'
  maya_lib_suffix = 'Maya.app/Contents/MacOS'
  maya_bin_suffix = 'Maya.app/Contents/bin'
  maya_plugin_ext = 'bundle'
  maya_compile_args += ['-DOSMac_']
  if meson.get_compiler('cpp').get_id() == 'clang'
    maya_compile_args += ['-std', 'c++0x', '-stdlib', 'libstdc++']
  endif
elif os_name == 'linux'
  maya_install_base = '/usr/autodesk'
  maya_plugin_ext = 'so'
  maya_compile_args += ['-DLINUX', '-fPIC']
else
  error('Incompatible operating system')
endif

maya_install_path = maya_install_base / ('Maya' + maya_version)

maya_bin_dir = maya_install_path / maya_bin_suffix
maya_bin = find_program('maya', dirs : maya_bin_dir)

maya_inc_dir = maya_install_path / maya_inc_suffix
maya_inc = include_directories(maya_inc_dir)

maya_lib_dir = maya_install_path / maya_lib_suffix
cmplr = meson.get_compiler('cpp')
maya_libs = [
  cmplr.find_library('Foundation', dirs : maya_lib_dir),
  cmplr.find_library('OpenMaya', dirs : maya_lib_dir),
  cmplr.find_library('OpenMayaAnim', dirs : maya_lib_dir),
  cmplr.find_library('OpenMayaFX', dirs : maya_lib_dir),
  cmplr.find_library('OpenMayaRender', dirs : maya_lib_dir),
  cmplr.find_library('OpenMayaUI', dirs : maya_lib_dir),
  cmplr.find_library('clew', dirs : maya_lib_dir),
]

maya_dep = declare_dependency(
  dependencies : maya_libs,
  include_directories : maya_inc,
  variables : {'name_suffix' : maya_plugin_ext, 'maya_version' : maya_version},
  compile_args : maya_compile_args,
  link_args : maya_link_args,

)

meson.override_dependency('maya', maya_dep)

