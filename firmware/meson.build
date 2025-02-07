project('piezo-control', 'cpp',
    default_options: [
        'cpp_std=c++20'
    ],
)

if not meson.is_cross_build()
    error('Must be a cross-compile environment.')
endif

boost_sml_inc = subproject('boost-sml').get_variable('boost_sml_inc')
cib_dep = subproject('compile-time-init-build').get_variable('cib_dep')
arduinocore_proj = subproject('arduinocore-avr')
adafruit_mcp4725_dep = subproject('Adafruit_MCP4725').get_variable('adafruit_mcp4725_dep')
encoder_dep = subproject('Encoder').get_variable('encoder_dep')

common_inc = include_directories('.')
subdir('pid-controller')

main_exe = executable('main',
    sources: [
        'main.cpp',
    ],
    cpp_args: [
        # For Encoder.h
        '-DARDUINO=182',

        # https://github.com/boost-ext/sml/issues/617
        '-DBOOST_SML_CREATE_DEFAULT_CONSTRUCTIBLE_DEPS',
    ],
    include_directories: [
        boost_sml_inc,
    ],
    dependencies: [
        cib_dep,
        # avr-ar does not support fat object files, why?
        arduinocore_proj.get_variable('arduinocore_dep'),
        arduinocore_proj.get_variable('arduinocore_wire_dep'),
        adafruit_mcp4725_dep,
        encoder_dep,
        pid_controller_dep,
    ],
)