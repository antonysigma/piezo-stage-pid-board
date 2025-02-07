pid_controller_lib = static_library('pid-controller',
    sources: 'src/pid-controller.cpp',
    include_directories: [
        'inc',
        common_inc,
    ],
    dependencies: [
        avr_stdcpp_dep,
    ],
)

pid_controller_dep = declare_dependency(
    link_with: pid_controller_lib,
    include_directories: 'inc',
)