# Warning! This code was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(app_sources
	"RB3202_DRV8833.cpp"
	"RB3202_dc_motor.cpp"
	"RB3202_encoder.cpp"
	"RB3202_stepr_motor.cpp"
	"main.cpp"
)

idf_component_register(SRCS ${app_sources})
