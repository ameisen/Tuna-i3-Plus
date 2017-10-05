require File.expand_path(File.dirname(__FILE__) + '/rbbuild/path_support.rb')
require 'open3'
require 'optparse'

$ARDUINO_BIN_PATH = best_path(ENV["ARDUINO_PATH"] + "hardware/tools/avr/bin", Dir.pwd) + "/"
$AVRDUDE_PATH = $ARDUINO_BIN_PATH + "avrdude.exe"
$AVR_CMD = "-C \"#{best_path(ENV["ARDUINO_PATH"] + "hardware/tools/avr/etc/avrdude.conf", Dir.pwd)}\" -c wiring -p atmega2560 -b 115200 -D -u"

$port = nil
$binary = nil

OptionParser.new do |opts|
	opts.banner = "Usage: deploy.rb [options]"
	
	opts.on("-p", "--port PORT", "Port to write to") { |v| $port = v }
	opts.on("-b", "--binary FILE", "Binary [required]") { |v| $binary = v }
end.parse!

if ($binary == nil)
	raise ArgumentError.new("User must pass a binary to the deploy script.")
end

if !(File.file? $binary)
	raise ArgumentError.new("Provided binary \"#{$binary}\" is not a valid file.")
end

#avrdude -cwiring -patmega2560 -b115200 -D -u -U flash:w:D:\Projects\i3PlusPlus\Marlin\out\tuna.elf.hex:i -P \\.\COM9

MaxCom = 256

def port_probe(comport)
	$stdout.sync = true
	STDOUT.flush
	
	stdin, stdout, stderr = Open3.popen3("\"" + $AVRDUDE_PATH + "\" " + $AVR_CMD + " -P " + comport + " -U \"" + "flash:w:#{$binary}:i" + "\"")
	stdin.close()
	stdout.sync = true
	stderr.sync = true
	errors = ""
	loop do
		char = stderr.getc
		if (char == nil)
			break
		end
		print char
		errors += char
	end
	stdout.close()
	stderr.close()
	if (errors.include? "bytes of flash verified")
		return true
	end
	STDOUT.flush
	return false
end

if ($port != nil)
	result = port_probe(comport)
	if (result == true)
		exit 0
	else
		puts "Failed to communicate via COM Port \"#{comport}\""
		exit 1
	end
end

puts "Probing COM Ports..."
(0..MaxCom).each { |port|
	comport = "\\\\.\\COM" + port.to_s
	if (!File.exist? comport)
		next
	end
	puts "Probing #{comport}"
	
	result = port_probe(comport)
	if (result == true)
		exit 0
	end
}

puts "Failed to communicate with any port."
exit 1

#C:\Program Files (x86)\Arduino\hardware\tools\avr\bin>avrdude -C %ARDUINO_PATH%/hardware/tools/avr/etc/avrdude.conf -c arduino -p atmega2560 \\.\com145
#avrdude: ser_open(): can't open device "\\.\com1": The system cannot find the file specified.
#
#
#avrdude done.  Thank you.