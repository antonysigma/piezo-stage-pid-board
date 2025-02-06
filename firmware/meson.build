project('piezo-control', 'cpp',
    default_options: [
        'cpp_std=c++20'
    ],
)

if not meson.is_cross_build()
    error('Must be a cross-compile environment.')
endif

cib_dep = subproject('compile-time-init-build').get_variable('cib_dep')
arduinocore_proj = subproject('arduinocore-avr')

main_exe = executable('main',
    sources: [
        'main2.cpp',
    ],
    dependencies: [
        cib_dep,
        # avr-ar does not support fat object files, why?
        arduinocore_proj.get_variable('arduinocore_dep'),
    ],
)