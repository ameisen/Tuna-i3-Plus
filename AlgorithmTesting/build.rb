$ARDUINO_BIN_PATH = ENV["ARDUINO_PATH"] + "hardware/tools/avr/bin/"

def gpp_path
	return "\"" + ($ARDUINO_BIN_PATH + "avr-g++.exe") + "\""
end

def gcc_path
	return "\"" + ($ARDUINO_BIN_PATH + "avr-gcc.exe") + "\""
end

def archiver_path
	return "\"" + ($ARDUINO_BIN_PATH + "avr-gcc-ar.exe") + "\""
end

def objcopy_path
	return "\"" + ($ARDUINO_BIN_PATH + "avr-objcopy.exe") + "\""
end

def buildline()
	buildopts = [
		"-O2",
		"-w",
		"-fpermissive",
		"-ffunction-sections",
		"-fdata-sections",
		"-fno-threadsafe-statics",
		"-fno-keep-static-consts",
		"-fmerge-all-constants",
		"-funsafe-loop-optimizations",
		"-fpredictive-commoning",
		"-mmcu=atmega2560",
		"-std=gnu++1z",
		"-fno-exceptions",
		"-DF_CPU=16000000L",
		"-DARDUINO=10804",
		"-DARDUINO_AVR_MEGA2560",
		"-DARDUINO_ARCH_AVR",
		"-I\"#{ENV["ARDUINO_PATH"] + "hardware/arduino/avr/cores/arduino"}\"",
		"-I\"#{ENV["ARDUINO_PATH"] + "hardware/arduino/avr/variants/mega"}\"",
		"-I.",
		"-isystem./tuna",
		"-fdelete-dead-exceptions",
		"-fshort-enums",
		"-freg-struct-return",
		"-fno-common",
		"-funswitch-loops",
		"-fgcse-after-reload",
		"-fsplit-paths",
		"-ftree-partial-pre",
		"-fgcse-sm",
		"-fgcse-las",
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

def compile_and_dump (source)
#-fverbose-asm
	command = gpp_path() + " -S -fverbose-asm " + buildline() + " -c #{source} -o #{source}.s"
	puts command
	STDOUT.flush
	
	pipe = IO.popen(command)
	output = pipe.readlines(nil)
	pipe.close
	if ($? != 0)
		raise RuntimeError.new("Failed to compile \"#{source}\" - return code #{$?.to_s}")
	end
end

Dir.foreach(".") do |entry|
	next if entry == '.' or entry == '..'
	next if File.extname(entry) != ".cpp"
	if (File.file? entry)
		# entry is source file
		compile_and_dump(entry)
	end
end