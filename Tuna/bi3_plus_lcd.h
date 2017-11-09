#ifndef BI3PLUSLCD_H
#define BI3PLUSLCD_H

#include "Marlin.h"
#include <tuna.h>

namespace tuna::lcd
{
	enum class Page : uint8
	{
		Boot_Animation = 1,
		// Boot Animation continues until 11
		Main_Menu = 11,
		Main_Menu_Pressed = 12,
		SD_Card = 31,
		SD_Card_Pressed = 32,
		Print = 33,
		Print_Pressed = 34,
		Print_Config = 35,
		Print_Config_Press = 36,
		Tool_Menu = 37,
		Tool_Menu_Press = 38,
		Preheat = 39,
		Preheat_Press = 40,
		Move = 41,
		Move_Press = 42,
		System_Menu = 43,
		System_Menu_Press = 44,
		PID = 45,
		PID_Press = 46,
		Motor = 47,
		Motor_Press = 48,
		Filament = 49,
		Filament_Press = 50,
		Unload_Filament = 51,
		Unload_Filament_Press = 52,
		Load_Filament = 53,
		Load_Filament_Press = 54,
		Level1 = 55,
		Level2 = 56,
		Level2_Press = 57,
		LCD_Update = 58,
		Statistics = 59,
		Statistics_Pressed = 60,
		Auto_PID = 61,
		Auto_PID_Pressed = 62,
		Temperature_Graph = 63,
		Temperature_Graph_Pressed = 64,
		Auto_PID_Graph = 65,
		PID_Finished = 66,
		PID_Finished_Pressed = 67,
		Thermal_Runaway = 68,
	};

	void initialize();
	void update();
	void show_page(Page pageNumber);
	void update_graph();
	constexpr inline bool has_status() { return false; }
	constexpr inline void set_status(const char* const, const bool = false) { }
	constexpr inline void set_status_PGM(const char* const, const int8_t = 0) { }
	constexpr inline void set_alert_status_PGM(const char*) { }
	constexpr inline void statusf(const uint8_t, const char * const, ...) { }
	constexpr inline void reset_alert_level() {}
	constexpr inline void refresh() {}
}

#define LCD_MESSAGEPGM(x)      lcd::set_status_PGM(PSTR(x))
#define LCD_ALERTMESSAGEPGM(x) lcd::set_alert_status_PGM(PSTR(x))

#endif
