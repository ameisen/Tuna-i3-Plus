#include "bi3_plus_lcd.h"

#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.hpp"
#include "stepper.h"
#include "configuration_store.h"
#include "utility.h"
#include "watchdog.h"

#if ENABLED(PRINTCOUNTER)
# include "printcounter.h"
# include "duration_t.h"
#endif

using namespace marlin::utils;

namespace marlin::lcd
{
	namespace
	{
		enum class OpMode : uint8
		{
			None = 0,
			Level_Init,
			Load_Filament,
			Unload_Filament,
			Move,
			Auto_PID
		};

		inline void read_data();
		void write_statistics();

		uint16 fileIndex = 0;
		millis_t nextOpTime = 0;
		millis_t nextLcdUpdate = 0;
		OpMode opMode = OpMode::None;
		uint8 eventCnt = 0;
		uint8 tempGraphUpdate = 0;
		uint8 lastPage = type_trait<uint8>::max;

		constexpr millis_t update_period = { 100 }; // originally 500

		void execute_looped_operation(millis_t ms)
		{
			if (ms < nextOpTime)
			{
				return;
			}
			switch (opMode)
			{
			case OpMode::Level_Init:
			{
				if (axis_homed[X_AXIS] & axis_homed[Y_AXIS] & axis_homed[Z_AXIS]) //stuck if levelling problem?
				{
					opMode = OpMode::None;
					show_page(56);//level 2 menu
				}
				else
				{
					nextOpTime = ms + 200;
				}
			} break;
			case OpMode::Unload_Filament:
			{
				if (Temperature::current_temperature >= (Temperature::target_temperature - 10))
				{
					enqueue_and_echo_commands_P(PSTR("G1 E-1 F120"));
				}
				nextOpTime = ms + 500;
			} break;
			case OpMode::Load_Filament:
			{
				if (Temperature::current_temperature >= (Temperature::target_temperature - 10))
				{
					enqueue_and_echo_commands_P(PSTR("G1 E1 F120"));
				}
				nextOpTime = ms + 500;
			} break;
			}
		}

		void status_update(millis_t ms)
		{
			if (ms < nextLcdUpdate)
			{
				return;
			}

			const millis_t difference = ms - nextLcdUpdate;

			nextLcdUpdate = ms + (update_period - min(difference, update_period));

			const auto target_hotend_temperature = Temperature::target_temperature;
			// TODO validate that float->uint conversion is correct
			const auto hotend_temperature = static_cast<int16>(roundf(Temperature::degHotend()));

			const auto target_bed_temperature = Temperature::target_temperature_bed;
			// TODO validate that float->uint conversion is correct
			const auto bed_temperature = static_cast<int16>(roundf(Temperature::degBed()));

			const auto fan_speed = (fanSpeeds[0] * 100) / 256;

			const auto card_progress = card.percentDone();

			const uint8 buffer[18] = {
				0x5A,
				0xA5,
				0x0F, //data length
				0x82, //write data to sram
				0x00, //starting at 0 vp
				0x00,
				hi(target_hotend_temperature), //0x00 target extruder temp
				lo(target_hotend_temperature),
				hi(hotend_temperature), //0x01 extruder temp
				lo(hotend_temperature),
				hi(target_bed_temperature), //0x02 target bed temp
				lo(target_bed_temperature),
				hi(bed_temperature), //0x03 target bed temp
				lo(bed_temperature),
				hi(fan_speed), //0x04 fan speed
				lo(fan_speed),
				0x00, //0x05 card progress
				card.percentDone()
			};

			serial<2>::write(buffer);

			switch (tempGraphUpdate)
			{
			case 2:
				tempGraphUpdate = 1;
				update_graph();
				break;
			default:
				tempGraphUpdate = 2;
				break;
			case 0:
				break;
			}
		}

		//show page OK
		uint8 get_current_page()
		{
			{
				constexpr const uint8 buffer[6] = {
					0x5A,//frame header
					0xA5,

					0x03,//data length

					0x81,//command - write read to register
					0x03,//register 0x03

					0x02,//2bytes
				};

				serial<2>::write(buffer);
			}

			uint8 buffer[8];
			const uint8 bytesRead = serial<2>::read_bytes(buffer);

			if ((bytesRead == 8) & (buffer[0] == 0x5A) & (buffer[1] == 0xA5))
			{
				return buffer[7];
			}

			return 0;
		}

		//receive data from lcd OK
		void read_data()
		{
			if (!serial<2>::available(1))
			{
				return;
			}

			if (serial<2>::read() != 0x5A)
			{
				return;
			}

			while (!serial<2>::available(1)) {}

			if (serial<2>::read() != 0xA5)
			{
				return;
			}

			while (!serial<2>::available(3)) {}

			serial<2>::read(); // data length
			serial<2>::read(); // command

			if (serial<2>::read() != 4) // VP MSB
			{
				return;
			}

			while (!serial<2>::available(4)) {}

			const uint8 lcdCommand = serial<2>::read(); // VP LSB
			serial<2>::read();// LEN ?
			serial<2>::read(); //KEY VALUE MSB
			const uint8 lcdData = serial<2>::read(); //KEY VALUE LSB

			switch (lcdCommand)
			{
			case 0x32: {//SD list navigation up/down OK
				if (card.sdprinting)
				{
					show_page(33); //show print menu
				}
				else
				{
					uint16 fileCnt = 0;
					if (lcdData == 0)
					{
						card.initsd();
						if (card.cardOK)
						{
							fileCnt = card.getnrfilenames();
							fileIndex = max(fileCnt, 1_u16) - 1;
						}
					}

					if (card.cardOK)
					{
						fileCnt = fileCnt ? fileCnt : card.getnrfilenames();
						card.getWorkDirName();//??

						if (fileCnt > 5)
						{
							if (lcdData == 1) //UP
							{
								if ((fileIndex + 5) < fileCnt)
								{
									fileIndex += 5;
								}
							}
							else if (lcdData == 2) //DOWN
							{
								if (fileIndex >= 5)
								{
									fileIndex -= 5;
								}
							}
						}

						{
							constexpr const uint8 buffer[6] = {
								0x5A,
								0xA5,
								0x9F,
								0x82,
								0x01,
								0x00
							};
							serial<2>::write(buffer);
						}

						for (uint8 i = 0; i < 6; ++i)
						{
							uint8 buffer[26];
							card.getfilename(fileIndex - i);
							serial<2>::write(card.longFilename, 26); // TODO why 26? No '\0'?
						}

						show_page(31); //show sd card menu
					}
				}
				break;
			}
			case 0x33: {//FILE SELECT OK
				if (card.cardOK) {
					if (((fileIndex + 10) - lcdData) >= 10)
					{
						card.getfilename(fileIndex - lcdData);

						constexpr const uint8 buffer[6] = {
							0x5A,
							0xA5,
							0x1D,
							0x82,
							0x01,
							0x4E
						};
						serial<2>::write(buffer);
						serial<2>::write(card.longFilename, 26);

						card.openFile(card.filename, true);
						card.startFileprint();
						print_job_timer.start();

						tempGraphUpdate = 2;

						show_page(33);//print menu
					}
				}
				break;
			}
			case 0x35: {//print stop OK
				card.stopSDPrint();
				clear_command_queue();
				quickstop_stepper();
				print_job_timer.stop();
				Temperature::disable_all_heaters();
#if FAN_COUNT > 0
				for (uint8 i = 0; i < FAN_COUNT; ++i)
				{
					fanSpeeds[i] = 0;
				}
#endif
				tempGraphUpdate = 0;
				show_page(11); //main menu
				break;
			}
			case 0x36: {//print pause OK
				card.pauseSDPrint();
				print_job_timer.pause();
#if ENABLED(PARK_HEAD_ON_PAUSE)
				enqueue_and_echo_commands_P(PSTR("M125"));
#endif
				break;
			}
			case 0x37: {//print start OK
#if ENABLED(PARK_HEAD_ON_PAUSE)
				enqueue_and_echo_commands_P(PSTR("M24"));
#else
				card.startFileprint();
				print_job_timer.start();
#endif
				break;
			}
			case 0x3C: { //Preheat options
				if (lcdData == 0) {
					//Serial.println(thermalManager.target_temperature[0]);
					//writing preset temps to lcd

					const int16 preset_hotend[3] = {
						planner.preheat_preset1_hotend,
						planner.preheat_preset2_hotend,
						planner.preheat_preset3_hotend
					};
					const int8 preset_bed[3] = {
						planner.preheat_preset1_bed,
						planner.preheat_preset2_bed,
						planner.preheat_preset3_bed
					};

					const uint8 buffer[18] = {
						 0x5A,
						 0xA5,
						 0x0F, //data length
						 0x82, //write data to sram
						 0x05, //starting at 0x0570 vp
						 0x70,
						 hi(preset_hotend[0]),
						 lo(preset_hotend[0]),
						 0x00,
						 preset_bed[0],
						 hi(preset_hotend[1]),
						 lo(preset_hotend[1]),
						 0x00,
						 preset_bed[1],
						 hi(preset_hotend[2]),
						 lo(preset_hotend[2]),
						 0x00,
						 preset_bed[2],
					};

					serial<2>::write(buffer);

					show_page(39);//open preheat screen
									//Serial.println(thermalManager.target_temperature[0]);
				}
				else {
					//Serial.println(thermalManager.target_temperature[0]);
					//read presets

					{
						constexpr const uint8 buffer[7] = {
							0x5A,
							0xA5,
							0x04, //data length
							0x83, //read sram
							0x05, //vp 0570
							0x70,
							0x06, //length
						};

						serial<2>::write(buffer);
					}

					//read user entered values from sram
					uint8 buffer[19];
					uint8 bytesRead = serial<2>::read_bytes(buffer);
					if ((bytesRead != 19) | (buffer[0] != 0x5A) | (buffer[1] != 0xA5))
					{
						break;
					}
					planner.preheat_preset1_hotend = int16{ buffer[7] } * 256_i16 + buffer[8];
					planner.preheat_preset1_bed = (int8)buffer[10];
					planner.preheat_preset2_hotend = int16{ buffer[11] } * 256_i16 + buffer[12];
					planner.preheat_preset2_bed = int8{ buffer[14] };
					planner.preheat_preset3_hotend = int16{ buffer[15] } * 256_i16 + buffer[16];
					planner.preheat_preset3_bed = int8{ buffer[18] };
					enqueue_and_echo_commands_P(PSTR("M500"));
					char command[20];
					switch (lcdData)
					{
					case 1: {
						//thermalManager.setTargetHotend(planner.preheat_preset1_hotend);
						//Serial.println(thermalManager.target_temperature[0]);
						sprintf(command, "M104 S%d", planner.preheat_preset1_hotend); //build heat up command (extruder)
						enqueue_and_echo_command((const char*)&command); //enque heat command
						sprintf(command, "M140 S%d", planner.preheat_preset1_bed); //build heat up command (bed)
						enqueue_and_echo_command((const char*)&command); //enque heat command
					} break;
					case 2: {
						sprintf(command, "M104 S%d", planner.preheat_preset2_hotend); //build heat up command (extruder)
						enqueue_and_echo_command((const char*)&command); //enque heat command
						sprintf(command, "M140 S%d", planner.preheat_preset2_bed); //build heat up command (bed)
						enqueue_and_echo_command((const char*)&command); //enque heat command
					} break;
					case 3: {
						sprintf(command, "M104 S%d", planner.preheat_preset3_hotend); //build heat up command (extruder)
						enqueue_and_echo_command((const char*)&command); //enque heat command
						sprintf(command, "M140 S%d", planner.preheat_preset3_bed); //build heat up command (bed)
						enqueue_and_echo_command((const char*)&command); //enque heat command
					} break;
					}
				}
			}
			case 0x34: {//cool down OK
				Temperature::disable_all_heaters();
				break;
			}
			case 0x3E: {//send pid/motor config to lcd OK

				const uint16 axis_steps_mm[4] = {
					uint16{ planner.axis_steps_per_mm[X_AXIS] * 10.0f },
					uint16{ planner.axis_steps_per_mm[Y_AXIS] * 10.0f },
					uint16{ planner.axis_steps_per_mm[Z_AXIS] * 10.0f },
					uint16{ planner.axis_steps_per_mm[E_AXIS] * 10.0f },
				};

				const uint16 Kp = uint16{ PID_PARAM(Kp) * 10.0f };
				const uint16 Ki = uint16{ unscalePID_i(PID_PARAM(Ki)) * 10.0f };
				const uint16 Kd = uint16{ unscalePID_d(PID_PARAM(Kd)) * 10.0f };

				const uint8 buffer[20] = {
					0x5A,
					0xA5,
					0x11,
					0x82,
					0x03,
					0x24,
					hi(axis_steps_mm[0]),
					lo(axis_steps_mm[0]),
					hi(axis_steps_mm[1]),
					lo(axis_steps_mm[1]),
					hi(axis_steps_mm[2]),
					lo(axis_steps_mm[2]),
					hi(axis_steps_mm[3]),
					lo(axis_steps_mm[3]),
					hi(Kp),
					lo(Kp),
					hi(Ki),
					lo(Ki),
					hi(Kd),
					lo(Kd),
				};

				serial<2>::write(buffer);

				show_page(lcdData ? 45 : 47); //show pid screen or motor screen
				break;
			}
			case 0x3F: {//save pid/motor config OK
				{
					constexpr const uint8 buffer[7] = {
						0x5A,
						0xA5,
						0x04,
						0x83,
						0x03,
						0x24,
						0x07
					};

					serial<2>::write(buffer);
				}

				uint8 buffer[21];
				uint8 bytesRead = serial<2>::read_bytes(buffer);
				if ((bytesRead != 21) | (buffer[0] != 0x5A) | (buffer[1] != 0xA5)) {
					break;
				}
				planner.axis_steps_per_mm[X_AXIS] = float{ ((uint16)buffer[7] * 256 + buffer[8]) } * 0.1f;
				//Serial.println(lcdBuff[7]);
				//Serial.println(lcdBuff[8]);
				//Serial.println(lcdBuff[9]);
				//Serial.println(lcdBuff[10]);
				planner.axis_steps_per_mm[Y_AXIS] = float{ ((uint16)buffer[9] * 256 + buffer[10]) } * 0.1f;
				planner.axis_steps_per_mm[Z_AXIS] = float{ ((uint16)buffer[11] * 256 + buffer[12]) } * 0.1f;
				planner.axis_steps_per_mm[E_AXIS] = float{ ((uint16)buffer[13] * 256 + buffer[14]) } * 0.1f;

				PID_PARAM(Kp) = float{ ((uint16)buffer[15] * 256 + buffer[16]) } * 0.1f;
				PID_PARAM(Ki) = scalePID_i(float{ ((uint16)buffer[17] * 256 + buffer[18]) } * 0.1f);
				PID_PARAM(Kd) = scalePID_d(float{ ((uint16)buffer[19] * 256 + buffer[20]) } * 0.1f);

				enqueue_and_echo_commands_P(PSTR("M500"));
				show_page(43);//show system menu
				break;
			}
			case 0x42: {//factory reset OK
				enqueue_and_echo_commands_P(PSTR("M502"));
				enqueue_and_echo_commands_P(PSTR("M500"));
				break;
			}
			case 0x47: {//print config open OK
				const int16 hotend_target = Temperature::degTargetHotend();
				const int16 bed_target = Temperature::degTargetBed();
				const int16 fan_speed = (fanSpeeds[0] * 100) / 256;

				const uint8 buffer[14] = {
					0x5A,
					0xA5,
					0x0B,
					0x82,
					0x03,
					0x2B,
					hi(feedrate_percentage), //0x2B
					lo(feedrate_percentage),
					hi(hotend_target), //0x2C
					lo(hotend_target),
					hi(bed_target), //0x2D
					lo(bed_target),
					hi(fan_speed),//0x2E
					lo(fan_speed)
				};

				serial<2>::write(buffer);

				show_page(35);//print config
				break;
			}
			case 0x40: {//print config save OK
				{
					constexpr const uint8 buffer[7] = {
						0x5A,
						0xA5,
						0x04,//4 byte
						0x83,//command
						0x03,// start addr
						0x2B,
						0x04, //4 vp
					};

					serial<2>::write(buffer);
				}

				uint8 buffer[15];
				uint8 bytesRead = serial<2>::read_bytes(buffer);
				if ((bytesRead != 15) | (buffer[0] != 0x5A) | (buffer[1] != 0xA5)) {
					break;
				}
				feedrate_percentage = (uint16)buffer[7] * 256 + buffer[8];
				Temperature::setTargetHotend((uint16)buffer[9] * 256 + buffer[10]);

				Temperature::setTargetBed(buffer[12]);
				fanSpeeds[0] = (uint16)buffer[14] * 256 / 100;
				show_page(33);// show print menu
				break;
			}
			case 0x4A: {//load/unload filament back OK
				opMode = OpMode::None;
				clear_command_queue();
				enqueue_and_echo_commands_P(PSTR("G90")); // absolute mode
				Temperature::setTargetHotend(0);
				show_page(49);//filament menu
				break;
			}
			case 0x4C: {//level menu OK
				switch (lcdData)
				{
				case 0: {
					show_page(55); //level 1
					axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
					enqueue_and_echo_commands_P(PSTR("G90")); //absolute mode
					enqueue_and_echo_commands_P((PSTR("G28")));//homeing
					nextOpTime = millis() + 200;
					opMode = OpMode::Level_Init;
				} break;
				case 1: { //fl
					enqueue_and_echo_commands_P((PSTR("G1 Z10 F2000")));
					enqueue_and_echo_commands_P((PSTR("G1 X30 Y30 F6000")));
					enqueue_and_echo_commands_P((PSTR("G1 Z0 F1000")));
				} break;
				case 2: { //rr
					enqueue_and_echo_commands_P((PSTR("G1 Z10 F2000")));
					enqueue_and_echo_commands_P((PSTR("G1 X170 Y170 F6000")));
					enqueue_and_echo_commands_P((PSTR("G1 Z0 F1000")));
				} break;
				case 3: { //fr
					enqueue_and_echo_commands_P((PSTR("G1 Z10 F2000")));
					enqueue_and_echo_commands_P((PSTR("G1 X170 Y30 F6000")));
					enqueue_and_echo_commands_P((PSTR("G1 Z0 F1000")));
				} break;
				case 4: { //rl
					enqueue_and_echo_commands_P((PSTR("G1 Z10 F2000")));
					enqueue_and_echo_commands_P((PSTR("G1 X30 Y170 F6000")));
					enqueue_and_echo_commands_P((PSTR("G1 Z0 F1000")));
				} break;
				case 5: { //c
					enqueue_and_echo_commands_P((PSTR("G1 Z10 F2000")));
					enqueue_and_echo_commands_P((PSTR("G1 X100 Y100 F6000")));
					enqueue_and_echo_commands_P((PSTR("G1 Z0 F1000")));
				} break;
				case 6: { //back
					enqueue_and_echo_commands_P((PSTR("G1 Z30 F2000")));
					show_page(37); //tool menu
				} break;
				}
				break;
			}

			case 0x51: { //load_unload_menu
				switch (lcdData)
				{
				case 0: {
					//writing default temp to lcd
					constexpr const uint8 buffer[8] = {
						0x5A,
						0xA5,
						0x05, //data length
						0x82, //write data to sram
						0x05, //starting at 0x0500 vp
						0x20,
						0x00,
						0xC8 //extruder temp (200)
					};
					serial<2>::write(buffer);

					show_page(49);//open load/unload_menu
				} break;
				case 1:
				case 2: {
					//read bed/hotend temp
					{
						constexpr const uint8 buffer[7] = {
							0x5A,
							0xA5,
							0x04, //data length
							0x83, //read sram
							0x05, //vp 0520
							0x20,
							0x01 //length
						};

						serial<2>::write(buffer);
					}

					//read user entered values from sram
					uint8 buffer[9];
					uint8 bytesRead = serial<2>::read_bytes(buffer);
					if ((bytesRead != 9) | (buffer[0] != 0x5A) | (buffer[1] != 0xA5)) {
						break;
					}
					int16 hotendTemp = (int16)buffer[7] * 256 + buffer[8];
					Serial.println(hotendTemp);
					char command[20];
					Temperature::setTargetHotend(hotendTemp);
					sprintf(command, "M104 S%d", hotendTemp); //build auto pid command (extruder)
																//enqueue_and_echo_command((const char*)&command); //enque pid command
					enqueue_and_echo_commands_P(PSTR("G91")); // relative mode
					nextOpTime = millis() + 500;
					if (lcdData == 1) {
						opMode = OpMode::Load_Filament;
					}
					else if (lcdData == 2) {
						opMode = OpMode::Unload_Filament;
					}
				} break;
				}
				break;
			}
			case 0x00: {
				clear_command_queue();
				enqueue_and_echo_commands_P(PSTR("G91"));
				enqueue_and_echo_commands_P(PSTR("G1 X5 F3000"));
				enqueue_and_echo_commands_P(PSTR("G90"));
				break;
			}
			case 0x01: {
				clear_command_queue();
				enqueue_and_echo_commands_P(PSTR("G91"));
				enqueue_and_echo_commands_P(PSTR("G1 X-5 F3000"));
				enqueue_and_echo_commands_P(PSTR("G90"));

				break;
			}
			case 0x02: {
				clear_command_queue();
				enqueue_and_echo_commands_P(PSTR("G91"));
				enqueue_and_echo_commands_P(PSTR("G1 Y5 F3000"));
				enqueue_and_echo_commands_P(PSTR("G90"));

				break;
			}
			case 0x03: {
				clear_command_queue();
				enqueue_and_echo_commands_P(PSTR("G91"));
				enqueue_and_echo_commands_P(PSTR("G1 Y-5 F3000"));
				enqueue_and_echo_commands_P(PSTR("G90"));

				break;
			}
			case 0x04: {
				clear_command_queue();
				enqueue_and_echo_commands_P(PSTR("G91"));
				enqueue_and_echo_commands_P(PSTR("G1 Z0.5 F3000"));
				enqueue_and_echo_commands_P(PSTR("G90"));

				break;
			}
			case 0x05: {
				clear_command_queue();
				enqueue_and_echo_commands_P(PSTR("G91"));
				enqueue_and_echo_commands_P(PSTR("G1 Z-0.5 F3000"));
				enqueue_and_echo_commands_P(PSTR("G90"));

				break;
			}
			case 0x06: {
				if (Temperature::degHotend() >= 180.0f) {
					clear_command_queue();
					enqueue_and_echo_commands_P(PSTR("G91"));
					enqueue_and_echo_commands_P(PSTR("G1 E1 F120"));
					enqueue_and_echo_commands_P(PSTR("G90"));
				}
				break;
			}
			case 0x07: {
				if (Temperature::degHotend() >= 180.0f) {
					clear_command_queue();
					enqueue_and_echo_commands_P(PSTR("G91"));
					enqueue_and_echo_commands_P(PSTR("G1 E-1 F120"));
					enqueue_and_echo_commands_P(PSTR("G90"));
				}
				break;
			}
			case 0x54: {//disable motors OK!!!
				enqueue_and_echo_commands_P(PSTR("M84"));
				axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
				break;
			}
			case 0x43: {//home x OK!!!
				enqueue_and_echo_commands_P(PSTR("G28 X0"));
				break;
			}
			case 0x44: {//home y OK!!!
				enqueue_and_echo_commands_P(PSTR("G28 Y0"));
				break;
			}
			case 0x45: {//home z OK!!!
				enqueue_and_echo_commands_P(PSTR("G28 Z0"));
				break;
			}
			case 0x1C: {//home xyz OK!!!
				enqueue_and_echo_commands_P(PSTR("G28"));
				break;
			}
			case 0x5B: { //stats menu
						 //sending stats to lcd
				write_statistics();

				show_page(59);//open stats screen on lcd
				break;
			}
			case 0x5C: { //auto pid menu

				if (lcdData == 0) {
					//writing default temp to lcd
					constexpr const uint8 buffer[8] = {
						0x5A,
						0xA5,
						0x05, //data length
						0x82, //write data to sram
						0x05, //starting at 0x0500 vp
						0x20,
						0x00,
						0xC8, //extruder temp (200)
					};
					serial<2>::write(buffer);

					show_page(61);//open auto pid screen
				}
				else if (lcdData == 1) { //auto pid start button pressed (1=hotend,2=bed)
										 //read bed/hotend temp

					{
						constexpr const uint8 buffer[7] = {
							0x5A,
							0xA5,
							0x04, //data length
							0x83, //read sram
							0x05, //vp 0520
							0x20,
							0x01, //length
						};
						serial<2>::write(buffer);
					}

					uint8 buffer[9];
					//read user entered values from sram
					uint8 bytesRead = serial<2>::read_bytes(buffer);
					if ((bytesRead != 9) | (buffer[0] != 0x5A) | (buffer[1] != 0xA5)) {
						break;
					}
					uint16 hotendTemp = (uint16)buffer[7] * 256 + buffer[8];
					//Serial.println(hotendTemp);
					char command[20];
					sprintf(command, "M303 S%d E0 C8 U1", hotendTemp); //build auto pid command (extruder)
					enqueue_and_echo_command("M106 S255"); //Turn on fan
					enqueue_and_echo_command(command); //enque pid command
					tempGraphUpdate = 2;
				}
				break;
			}
			case 0x3D: { //Close temp screen
				if (lcdData == 1) {
					tempGraphUpdate = 0;
					Serial.println(lastPage);
					show_page(lastPage);
				}
				else {
					lastPage = get_current_page();
					Serial.println(lastPage);
					tempGraphUpdate = 2;
					show_page(63);
				}
			}
			case 0x55: { //enter print menu without selecting file
				tempGraphUpdate = 2;
				if (card.sdprinting == false)
				{
					constexpr const uint8 buffer[6] = {
						0x5A,
						0xA5,
						0x1D,
						0x82,
						0x01,
						0x4E
					};
					serial<2>::write(buffer);
					constexpr const char str_buffer[26] = "No SD print";
					serial<2>::write(str_buffer);
				}
				show_page(33);//print menu
			}
					   /*case 0xFF: {
					   show_page(58); //enable lcd bridge mode
					   while (1) {
					   watchdog_reset();
					   if (Serial.available())
					   Serial2.write(Serial.read());
					   if (Serial2.available())
					   Serial.write(Serial2.read());
					   }
					   break;
					   }*/
			default:
				break;
			}
		}

		void lcdSendMarlinVersion()
		{
			struct final
			{
				const uint8 buffer[6] = {
					0x5A,
					0xA5,
					0x12,
					0x82,
					0x05,
					0x00
				};
				const char version_str[15] = SHORT_BUILD_VERSION;
			} constexpr const version_data;

			serial<2>::write_struct(version_data);
		}

		void write_statistics()
		{
			printStatistics stats = print_job_timer.getStats();

			{
				//Total prints (including aborted)
				const uint16 totalPrints = stats.totalPrints;
				//Finished prints
				const uint16 finishedPrints = stats.finishedPrints;

				const uint8 buffer[10] = {
					0x5A, 
					0xA5, 
					0x07, //data length
					0x82, //write data to sram
					0x05,  //starting at 0x5040 vp
					0x40, 
					hi(totalPrints),
					lo(totalPrints),
					hi(finishedPrints),
					lo(finishedPrints)
				};

				serial<2>::write(buffer);
			}

			{
				struct final
				{
					const uint8 buffer[6] = {
						0x5A,
						0xA5,
						0x12,
						0x82,
						0x05,
						0x42
					};
					char time_str[15];
				} time_data;

				duration_t{ stats.printTime }.toString<15>(time_data.time_str);

				serial<2>::write_struct(time_data);
			}

			{
				//longest print time
				struct final
				{
					const uint8 buffer[6] = {
						0x5A,
						0xA5,
						0x12,
						0x82,
						0x05,
						0x4D
					};
					char time_str[15];
				} time_data;

				duration_t{ stats.longestPrint }.toString<15>(time_data.time_str);

				serial<2>::write_struct(time_data);
			}

			{
				//total filament used
				struct final
				{
					const uint8 buffer[6] = {
						0x5A,
						0xA5,
						0x12, //data length
						0x82, //write data to sram
						0x05, //starting at 0x0558 vp
						0x58
					};
					char filament_str[15];
				} filament_data;

				snprintf_P(filament_data.filament_str, sizeof(filament_data.filament_str), PSTR("%ld.%im"), long(stats.filamentUsed / 1000.0), int(stats.filamentUsed / 100.0) % 10);

				serial<2>::write_struct(filament_data);
			}
		}
	}

	//init OK
	void initialize()
	{
		serial<2>::begin<115'200_u32>();

		lcdSendMarlinVersion();
		show_page(0x01);
	}

	//lcd status update OK
	void update()
	{
		read_data();

		const millis_t ms = millis();
		execute_looped_operation(ms);
		status_update(ms);
	}

	//show page OK
	void show_page(uint8 pageNumber)
	{
		const uint8 buffer[7] = {
			0x5A,//frame header
			0xA5,
			0x04,//data length
			0x80,//command - write data to register
			0x03,
			0x00,
			pageNumber
		};

		serial<2>::write(buffer);
	}

	void update_graph() {
		const int16 hotend = static_cast<int16>(roundf(Temperature::degHotend()));
		const int16 bed = static_cast<int16>(roundf(Temperature::degBed()));

		auto foo = type_trait<int16>::unsigned_type{ 0 };

		const uint8 buffer[9] = {
			0x5A,
			0xA5,
			0x06, //data length
			0x84, //update curve
			0x03, //channels 0,1
			hi(hotend), //TODOME
			lo(hotend), //TODOME
			hi(bed),
			lo(bed)
		};

		serial<2>::write(buffer);
	}
}
