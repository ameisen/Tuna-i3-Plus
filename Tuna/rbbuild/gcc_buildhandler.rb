$script_mtime = [$script_mtime, File.mtime(__FILE__).to_f].max

require_relative 'path_support.rb'

$ARDUINO_BIN_PATH = best_path(ENV["ARDUINO_PATH"] + "hardware/tools/avr/bin", Dir.pwd) + "/"

$gpp_buildhandler = Class.new do
	def self.name
		return "gcc/g++ build handler"
	end
	
	def self.extensions
		return [".cpp", ".cxx", ".cc", ".c++", ".c", ".gcpp"];
	end
	
	def self.is_c(source)
		return [".c"].include? File.extname(source).downcase
	end

	def self.is_gccp(source)
		return [".gcpp"].include? File.extname(source).downcase
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
			is_gccp(source) ? "-x c++" : "",
			"-O3",
			"-w",
			"-fpermissive",
			"-ffunction-sections",
			"-fdata-sections",
			"-fno-threadsafe-statics",
			"-flto",
			"-fno-fat-lto-objects",
			"-fno-keep-static-consts",
			"-fmerge-all-constants",
			"-funsafe-loop-optimizations",
			"-fpredictive-commoning",
			"-mmcu=atmega2560",
			is_c(source) ? "-std=gnu11" : "-std=gnu++17",
			is_c(source) ? "" : "-fno-exceptions",
			"-DF_CPU=16000000L",
			"-DARDUINO=10804",
			"-DARDUINO_AVR_MEGA2560",
			"-DARDUINO_ARCH_AVR",
			"-I#{quote_wrap(best_path(ENV["ARDUINO_PATH"] + "hardware/arduino/avr/cores/arduino", Dir.pwd))}",
			"-I#{quote_wrap(best_path(ENV["ARDUINO_PATH"] + "hardware/arduino/avr/variants/mega", Dir.pwd))}",
			"-I.",
			"-isystem./tuna",
			"-fdelete-dead-exceptions",
			"-fshort-enums", #-fno-short-enums is for some reason dramatically smaller. Investigate.
			"-freg-struct-return",
			"-fno-common",
			"-fno-unswitch-loops", #-funswitch-loops makes worse code on AVR.
			"-fgcse-after-reload",
			"-ftree-loop-vectorize", # not sure why, but this generates better code?
			"-fsplit-paths", #-fno-split-paths generates smaller code. Investigate if the code is actually better.
			"-ftree-partial-pre",
			"-fno-gcse-sm",
			"-fno-gcse-las",
			"-fgcse-after-reload",
			"-fdeclone-ctor-dtor",
			"-fdevirtualize-speculatively",
			"-fdevirtualize-at-ltrans",
			"-free",
			"-fsched-pressure",
			"-fsched-spec-load",
			"-fipa-pta",
			"-ftree-builtin-call-dce",
			"-ftree-loop-distribute-patterns",
			"-ftree-loop-ivcanon",
			"-fivopts",
			"-fvariable-expansion-in-unroller",
			"-fno-align-functions",
			"-fno-align-labels",
			"-fno-align-loops",
			"-fno-align-jumps",
			"-fassociative-math",
			"-freciprocal-math",
			"-fbranch-target-load-optimize",
			"-fbranch-target-load-optimize2",
			"-fstdarg-opt"
		]
		
		# -fdelete-dead-exceptions
		# -fshort-enums
		# -freg-struct-return
		# -fno-common
		# 
		
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
			STDOUT.flush
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("objcopy command failed - return code #{$?.to_s}")
		end
	end
	
	def self.compile(source, out, include_paths, print_cmd = true)		
		includes = ""
		include_paths.each { |path|
			includes += "-I#{quote_wrap(path)} "
		}
		includes.chomp!
		command = compiler_path(source) + " " + includes + " " + buildline(source) + " -c #{quote_wrap(source)} -o #{quote_wrap(out)}"
		if (print_cmd)
			puts $TAB + command
			STDOUT.flush
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
			STDOUT.flush
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
	
	def self.archive(archive, objects, print_cmd = true)
		@MaxCmd = 8191
	
		baseCmd = archiver_path() + " rcs " + quote_wrap(archive) + " ";
		
		commands = []
		command = baseCmd.dup
		objects.each { |object|
			object = quote_wrap(object.object_path)
			if ((command.length + object.length) >= @MaxCmd)
				# Flush command and start a new one.
				commands << command.dup.chop
				command = baseCmd.dup
			end
			command = command + object + " "
		}
		if (command != baseCmd)
			commands << command.dup.chop
		end
	
		commands.each { |command|
			if (print_cmd)
				puts $TAB + command
				STDOUT.flush
			end
			
			pipe = IO.popen(command)
			output = pipe.readlines(nil)
			pipe.close
			if ($? != 0)
				raise RuntimeError.new("Failed to archive \"#{object}\" - return code #{$?.to_s}")
			end
		}
	end
	
	def self.link(outfile, archive, library_paths, print_cmd = true)
	
		lib_str = ""
		library_paths.each { |dir|
			lib_str += "-L#{quote_wrap(dir)} "
		}

		command = gcc_path() + " " + buildline() +  "-Wl,--sort-common -s -fuse-linker-plugin -Wl,--gc-sections,--relax -o \"#{outfile}\" \"#{archive}\" #{lib_str}-lm"
		if (print_cmd)
			puts $TAB + command
			STDOUT.flush
		end
		
		pipe = IO.popen(command)
		output = pipe.readlines(nil)
		pipe.close
		if ($? != 0)
			raise RuntimeError.new("Failed to link \"#{outfile}\" - return code #{$?.to_s}")
		end
	end
end
