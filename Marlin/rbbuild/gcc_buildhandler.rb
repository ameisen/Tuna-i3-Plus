require File.expand_path(File.dirname(__FILE__) + '/path_support.rb')

$ARDUINO_BIN_PATH = best_path(ENV["ARDUINO_PATH"] + "hardware/tools/avr/bin", Dir.pwd) + "/"

$gpp_buildhandler = Class.new do
	def self.name
		return "gcc/g++ build handler"
	end
	
	def self.extensions
		return [".cpp", ".cxx", ".cc", ".c++", ".c"];
	end
	
	def self.is_c(source)
		return [".c"].include? File.extname(source).downcase
	end
	
	def self.quote_wrap(str)
		if (str =~ /\s/)
			return "\"" + str + "\""
		end
		return str
	end
	
	def self.compiler_path(source)
		return is_c(source) ? gcc_path() : gpp_path()
	end
	
	def self.gpp_path
		return quote_wrap($ARDUINO_BIN_PATH + "avr-g++.exe")
	end
	
	def self.gcc_path
		return quote_wrap($ARDUINO_BIN_PATH + "avr-gcc.exe")
	end
	
	def self.archiver_path
		return quote_wrap($ARDUINO_BIN_PATH + "avr-gcc-ar.exe")
	end
	
	def self.objcopy_path
		return quote_wrap($ARDUINO_BIN_PATH + "avr-objcopy.exe")
	end
	
	def self.buildline(source = "dummy.cpp")
		buildopts = [
			"-O2",
			"-w",
			"-fpermissive",
			"-ffunction-sections",
			"-fdata-sections",
			"-fno-threadsafe-statics",
			"-flto",
			"-fno-keep-static-consts",
			"-fmerge-all-constants",
			"-funsafe-loop-optimizations",
			"-fpredictive-commoning",
			"-mmcu=atmega2560",
			is_c(source) ? "-std=gnu11" : "-std=gnu++1z",
			is_c(source) ? "-fno-exceptions" : "",
			"-DF_CPU=16000000L",
			"-DARDUINO=10804",
			"-DARDUINO_AVR_MEGA2560",
			"-DARDUINO_ARCH_AVR",
			"-I#{quote_wrap(best_path(ENV["ARDUINO_PATH"] + "hardware/arduino/avr/cores/arduino", Dir.pwd))}",
			"-I#{quote_wrap(best_path(ENV["ARDUINO_PATH"] + "hardware/arduino/avr/variants/mega", Dir.pwd))}",
			"-I.",
			"-isystem./tuna"
		]
		
		return_opts = ""
		buildopts.each { |opt|
			if (opt.length)
				return_opts += opt + " "
			end
		}
		return return_opts.chomp
	end
	
	def self.objcopy(command, print_cmd = true)
		command = objcopy_path + " " + command
		if (print_cmd)
			puts $TAB + command
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("objcopy command failed - return code #{$?.to_s}")
		end
	end
	
	def self.compile(source, out, print_cmd = true)		
		command = compiler_path(source) + " " + buildline(source) + " -c #{quote_wrap(source)} -o #{quote_wrap(out)}"
		if (print_cmd)
			puts $TAB + command
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to compile \"#{source}\" - return code #{$?.to_s}")
		end
	end
	
	def self.get_dependencies(source, print_cmd = false)
		command = compiler_path(source) + " " + buildline(source) + " -M -MT \"\" #{quote_wrap(source)}"
		if (print_cmd)
			puts $TAB + command
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to get source dependencies for \"#{source}\" - return code #{$?.to_s}")
		end
		output = output.join(" ").strip.gsub(/\r/,"").gsub(/\n/,"").gsub(/ \\ /," ")
		output = output[2..-1].split(' ')[1..-1]
		clean_paths(output)
		
		return output.uniq
	end
	
	def self.archive(archive, object, print_cmd = true)
		command = archiver_path() + " rcs " + quote_wrap(archive) + " " + quote_wrap(object)
		if (print_cmd)
			puts $TAB + command
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to archive \"#{object}\" - return code #{$?.to_s}")
		end
	end
	
	def self.link(outfile, archive, library_paths, print_cmd = true)
	
		lib_str = ""
		library_paths.each { |dir|
			lib_str += "-L#{quote_wrap(dir)} "
		}

		command = gcc_path() + " " + buildline() +  "-fuse-linker-plugin -Wl,--gc-sections,--relax -o \"#{outfile}\" \"#{archive}\" #{lib_str}-lm"
		if (print_cmd)
			puts $TAB + command
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to link \"#{outfile}\" - return code #{$?.to_s}")
		end
	end
end
