global_time = Time.now
init_time = Time.now

require 'fileutils'
require 'pathname'
require File.expand_path(File.dirname(__FILE__) + '/path_support.rb')
require File.expand_path(File.dirname(__FILE__) + '/buildopts.rb')
require File.expand_path(File.dirname(__FILE__) + '/gcc_buildhandler.rb')
require File.expand_path(File.dirname(__FILE__) + '/thread_exec.rb')

def duration (timediff)
	out = ""
	
	days = (timediff / 86400.0).floor
	timediff = timediff % 86400.0
	hours = (timediff / 3600.0).floor
	timediff = timediff % 3600.0
	minutes = (timediff / 60.0).floor
	timediff = timediff % 60.0
	seconds = timediff
	
	if (days > 0)
		out += days.to_s + "d "
	end
	if (hours > 0)
		out += hours.to_s + "h "
	end
	if (minutes > 0)
		out += minutes.to_s + "m "
	end
	out += seconds.to_s + "s"
	
	return out;
end

# Print Build Configuration
$BuildOptions.print_self()

force_all_build = $BuildOptions.target == "rebuild"
force_clean = ($BuildOptions.target == "rebuild") || ($BuildOptions.target == "clean")

def str_hash (str)
	hash = 14695981039346656037
	str.split('').each { |c|
		hash = (hash * 1099511628211) & 0xFFFFFFFFFFFFFFFF
		hash = (hash ^ c.to_i)
	}
	return hash
end

def vputs(str, tabs = 0)
	if ($BuildOptions.verbose)
		puts(($TAB * tabs) + "VERBOSE: " + str)
	end
end

# Build Handler setup
$source_files = Hash.new nil
$source_handlers = Hash.new nil
Handlers = [
	$gpp_buildhandler
]

Handlers.each { |handler|
	handler.extensions.each { |ext|
		$source_handlers[ext.downcase] = handler
	}
	$source_files[handler] = []
}

init_time = Time.now - init_time

puts "Build System Initialized (#{duration(init_time)})";

def force_delete(path)
	if (File.directory? path)
		FileUtils.rm_rf(path)
		if (File.directory? path)
			FileUtils.remove_dir(path)
		end
	elsif (File.file? path)
		File.delete(path)
	end
end

if (force_clean)
	clean_time = Time.now
	puts "Cleaning..."
	
	# remove any output objects
	
	if (File.file? $BuildOptions.output)
		File.delete($BuildOptions.output)
	end
	$BuildOptions.output_deps.each { |dep|
		next if dep == '.' or dep == '..'
		if (File.file? dep)
			force_delete(dep)
		end
	}
	force_delete(File.dirname($BuildOptions.output))
	
	# delete all intermediates
	
	dir = $BuildOptions.intermediate

	if (File.directory? dir)
		def parse_directory(dir)
			Dir.foreach(dir) do |entry|
				next if entry == '.' or entry == '..'
				entry = dir + "/" + entry
				
				if (File.directory? entry)
					parse_directory(File.expand_path(entry))
					force_delete(entry)
				elsif (File.file? entry)
					force_delete(entry)
				end
			end
		end
		parse_directory(dir)
		force_delete(dir)
	end
	
	
	clean_time = Time.now - clean_time
	puts "Clean Complete (#{duration(clean_time)})"
	
	if ($BuildOptions.target == "clean")
		exit 0
	end
end

parse_time = Time.now

# Parse source directories
$directory_hashes = Hash.new nil

def push_directory_hash(dir)
	hash = $directory_hashes[dir]
	if (hash != nil)
		return hash
	end
	hash = str_hash(dir)
	$directory_hashes[dir] = hash
	return hash
end

class Source
	attr_accessor :path
	attr_accessor :hash
	attr_accessor :handler
	attr_accessor :build
	
	def initialize (params = {})
		@path = params.fetch(:path)
		@hash = params.fetch(:hash)
		@handler = params.fetch(:handler)
		@build = false
	end
	
	def object_path
		base = File.basename(path)
		ipath = $BuildOptions.intermediate + "/" + hash.to_s(36) + "/" + base + ".o"
		return best_path(ipath)
	end
end

$BuildOptions.src_dirs.each { |dir|
	dir = File.expand_path(dir)

	def parse_directory(dir)
		Dir.foreach(dir) do |entry|
			next if entry == '.' or entry == '..'
			entry = dir + "/" + entry
			#we will store relative paths generally, however, as they are often shorter and put less pressure on the command line, especially on Windows.
			if (File.directory? entry)
				parse_directory(File.expand_path(entry))
			elsif (File.file? entry)
				hash = push_directory_hash(dir) # would do this higher, but it's easier to stop empty directories here
				extension = File.extname(entry).downcase
				next if $source_handlers[extension] == nil
				path = best_path(entry)
				source_file = Source.new( path: path, hash: hash, handler: $gpp_buildhandler )
				$source_files[$gpp_buildhandler] << source_file;
			end
		end
	end
	parse_directory(dir)
}

# Generate the hashed intermediate directories.
FileUtils.mkdir_p $BuildOptions.intermediate
$directory_hashes.each { |path, hash|
	FileUtils.mkdir_p $BuildOptions.intermediate + "/" + hash.to_s(36)
}

parse_time = Time.now - parse_time
puts("Directories Parsed. (#{duration(parse_time)})");

# Now we iterate over our source files, and generate header dependency lists for them.
# TODO we can optimize this so it doesn't have to happen every time by cacheing results.
# TODO also we can thread this

dep_time = Time.now

newest_objtime = 0.0
any_rebuild = false
puts "Determining which object files need to be rebuilt"
$dependency_times = Hash.new nil
$source_files.each { |handler, files|
	Threader.new(
		-> (idx) {
			src = files[idx]
			
			src_time = File.mtime(src.path).to_f
			obj_time = 0.0
			
			# First, we will check if an object file exists and if it's older than the source file.
			obj_path = src.object_path()
			if (File.file? obj_path)
				obj_time = File.mtime(obj_path).to_f
				newest_objtime = [obj_time, newest_objtime].max
				if (obj_time < src_time)
					# obj file older than source
					vputs("\"#{src.path}\" being rebuilt because object file \"#{obj_path}\" is older than source file", 1)
					src.build = true
					any_rebuild = true
					next
				end
			else
				# obj file doesn't exist
				vputs("\"#{src.path}\" being rebuilt because object file \"#{obj_path}\" doesn't exist", 1)
				src.build = true
				any_rebuild = true
				next
			end
			vputs("\"#{src.path}\" object file \"#{obj_path}\" is fine", 1)
			
			# Second, if we get here, we will check the dependencies to see if any of them are newer than the object file.
			dependencies = handler.get_dependencies(src.path)
			dependencies.each { |dep|
				deptime = $dependency_times[dep]
				if (deptime == nil)
					deptime = File.mtime(dep).to_f
					$dependency_times[dep] = deptime
				end
				if (deptime > obj_time)
					# A dependency is newer than the source file. It needs to be rebuilt.
					vputs("\"#{src.path}\" being rebuilt because object file is older than dependency \"#{dep}\"", 1)
					src.build = true
					any_rebuild = true
					break
				end
			}
		},
	files.length).join()
}

dep_time = Time.now - dep_time
puts "Dependency Generation Complete. (#{duration(dep_time)})"
compile_time = Time.now

if (any_rebuild)
	puts "The following files will be compiled:"
	$source_files.each { |handler, files|
		files.each { |src|
			if (src.build)
				puttabs(src.path, 1);
			end
		}
	}
	
	puts "Compiling..."
	$source_files.each { |handler, files|
	
		Threader.new(
		-> (idx) {
			src = files[idx]
			
			if (src.build)
				handler.compile(src.path, src.object_path())
			end
		},
		files.length).join()
	}
	compile_time = Time.now - compile_time
	puts "Compilation Complete. (#{duration(compile_time)})"
else
	compile_time = Time.now - compile_time
	puts "No object files are out of date. (#{duration(compile_time)})"
end

archiving_time = Time.now

# Now we generate an archive.
archive_path = best_path($BuildOptions.intermediate + "/" + $BuildOptions.name + ".a")
build_archive = any_rebuild
if (!build_archive)
	if (File.file? archive_path)
		archive_time = File.mtime(archive_path).to_f
		if (archive_time < newest_objtime)
			build_archive = true
		end
	else
		build_archive = true
	end
end
if (build_archive)
	any_rebuild = true
	puts "Regenerating archive file."
	if (File.file? archive_path)
		File.delete(archive_path)
	end
	
	$source_files.each { |handler, files|
		files.each { |src|
			obj_path = src.object_path
			handler.archive(archive_path, obj_path)
		}
	}
	archiving_time = Time.now - archiving_time
	puts "Archiving Complete. (#{duration(archiving_time)})"
else
	archiving_time = Time.now - archiving_time
	puts "Archive file is not out of date. (#{duration(archiving_time)})"
end

link_time = Time.now

# Now we link.
FileUtils.mkdir_p File.dirname($BuildOptions.output)
binary_path = $BuildOptions.output
binary_rebuild = any_rebuild || build_archive
if (!binary_rebuild)
	binary_exist = File.file? binary_path
	if (!binary_exist)
		binary_rebuild = true
	else
		archive_time = File.mtime(archive_path).to_f
		binary_time = File.mtime(binary_path).to_f
		if (archive_time > binary_time)
			binary_rebuild = true
		end
	end
end

if (binary_rebuild)
	puts "Output Binary is out of date, relinking."
	$gpp_buildhandler.link(binary_path, archive_path, [])
	any_rebuild = true
	link_time = Time.now - link_time
	puts "Linking Complete (#{duration(link_time)})"
else
	link_time = Time.now - link_time
	puts "Output binary is not out of date. (#{duration(link_time)})"
end

custom_time = Time.now

puts "Beginning Custom Build Step."

def out_of_date (object, reference)
	if !(File.file? object)
		return true
	end
	object_time = File.mtime(object).to_f
	reference_time = File.mtime(reference).to_f
	return object_time < reference_time
end

##### CUSTOM BUILD STEPS #####
$BuildOptions.output_deps.each { |dep|
	FileUtils.mkdir_p File.dirname(dep)
}

def quote_wrap(str)
	if (str =~ /\s/)
		return "\"" + str + "\""
	end
	return str
end

eep_path = File.dirname($BuildOptions.output) + "/" + File.basename($BuildOptions.output) + ".eep"
eep_ood = out_of_date(eep_path, $BuildOptions.output);
if (!eep_ood)
	puttabs "EEP file is not out of date."
else
	any_rebuild = true
	puttabs "EEP file is out of date - rebuilding."
	command = "-O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 #{quote_wrap($BuildOptions.output)} #{quote_wrap(eep_path)}"
	$gpp_buildhandler.objcopy(command)
	# avr-objcopy" -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0  "C:\Users\mkuklinski\AppData\Local\Temp\arduino_build_225425/Marlin.ino.elf" "C:\Users\mkuklinski\AppData\Local\Temp\arduino_build_225425/Marlin.ino.eep"
end

hex_path = File.dirname($BuildOptions.output) + "/" + File.basename($BuildOptions.output) + ".hex"
hex_ood = out_of_date(hex_path, $BuildOptions.output);
if (!hex_ood)
	puttabs "HEX file is not out of date."
else
	any_rebuild = true
	puttabs "HEX file is out of date - rebuilding."
	command = "-O ihex -R .eeprom #{quote_wrap($BuildOptions.output)} #{quote_wrap(hex_path)}"
	$gpp_buildhandler.objcopy(command)
	# avr-objcopy" -O ihex -R .eeprom  "C:\Users\mkuklinski\AppData\Local\Temp\arduino_build_225425/Marlin.ino.elf" "C:\Users\mkuklinski\AppData\Local\Temp\arduino_build_225425/Marlin.ino.hex"
end

custom_time = Time.now - custom_time
puts "Custom Build Step Complete (#{duration(custom_time)})"
global_time = Time.now - global_time

if (any_rebuild)
	puts "Build Complete. (#{duration(global_time)})"
else
	puts "Build Not Needed. (#{duration(global_time)})"
end