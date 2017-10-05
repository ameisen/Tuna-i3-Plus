require 'optparse'
require 'etc'
require_relative 'path_support.rb'
require_relative 'gcc_buildhandler.rb'
require_relative 'ascii.rb'

$environment = []

$BuildOptions = Class.new do
	class << self
		attr_accessor :name
		attr_accessor :target
		attr_accessor :root
		attr_accessor :intermediate
		attr_accessor :output
		attr_accessor :output_deps
		attr_accessor :src_dirs
		attr_accessor :threads
		attr_accessor :verbose
	end
	@name = nil
	@target = nil
	@root = File.expand_path(normalize_path(Dir.pwd))
	@intermediate = nil
	@output = nil
	@output_deps = []
	@src_dirs = []
	@threads = nil
	@verbose = false
	
	def self.validate
		errors = []

		if !(File.directory? root)
			errors << "Root Directory \'#{root.to_str}\' is not a valid directory."
		end
		Dir.chdir root
		
		if (@name == nil || @name == "")
			errors << "Must provide a build name."
		end

		if (@target == nil || @target == "")
			errors << "Must provide a build target."
		end
		
		if (!($build_targets.include? @target))
			errors << "Build Target \'#{@target}\' is not a legal build target {#{build_target_str()}}"
		end
		
		if (@intermediate == nil || @intermediate == "")
			errors << "Must provide a path for intermediate files."
		end
		
		if (@output == nil || @output == "")
			errors << "Must provide an output file."
		end
		
		if (@src_dirs.size == 0)
			errors << "Must provide at least one source directory."
		end
		
		if (@threads != nil && @threads <= 0)
			errors << "The number of threads must be a positive integer greater than 0"
		elsif (@threads == nil)
			@threads = Etc.nprocessors
		end
		
	
		@src_dirs.each { |path|
			if !(File.directory? path)
				errors << "Source Directory \'#{path.to_str}\' is not a valid directory."
			end
		}
		
		if (errors.count != 0)
			error_str = ""
			errors.each { |e|
				error_str += e
				error_str += ' ';
			}
			return error_str.chop
		end
		
		# Finalize some of the values.
		@intermediate = best_path(@intermediate)
		clean_paths(@src_dirs)
		clean_paths(@output_deps)
		@output = best_path(@output)
		
		if (File.dirname(@output) == ".")
			errors << "Output must not be in the root directory."
		end
		
		if (@intermediate == ".")
			errors << "Intermediate directory must not be the root directory."
		end
		
		if (errors.count != 0)
			error_str = ""
			errors.each { |e|
				error_str += e
				error_str += ' ';
			}
			return error_str.chop
		end
		
		return nil
	end
	
	def self.print_self()
		to_print = Hash.new
		to_print["Name"] = @name;
		to_print["Target"] = @target;
		to_print["Root"] = @root;
		to_print["Threads"] = @threads;
		to_print["Intermediate Path"] = @intermediate;
		to_print["Output"] = @output;
		to_print["Output Dependencies"] = @output_deps;
		to_print["Source Paths"] = @src_dirs;
		
		max_length = 0
		
		@Terminator ||= " #{$Ascii['═']} "
		@Array_Unique ||= " #{$Ascii['═']} "
		@Array_Kicker ||= " #{$Ascii['╒']} "
		@Array_Line ||= " #{$Ascii['│']} "
		@Array_Footer ||= " #{$Ascii['╘']} "
		if !([@Terminator, @Array_Kicker, @Array_Line, @Array_Footer].all? {|v| v.length == @Terminator.length})
			#raise SyntaxError.new("Configuration Print Terminators are not equivalent in length.")
		end
		
		to_print.each { |name, src|
			max_length = [max_length, name.length].max
		}
		
		max_length += $TAB.length
		
		puts("Build Configuration:")
		to_print.each { |name, src|
			name = puttab(name)
			len = name.length
			len_adjust = max_length - len;
			name = name + (" " * len_adjust)
			if (src.kind_of? Array)
				if (src.size != 0)
					if (src.size == 1)
						puts(name + @Array_Unique + src.last)
					else
						print(name + @Array_Kicker)
						src.each { |val|
							header = ""
							if (val == src.first)
								puts(val)
								next
							elsif (val == src.last)
								header = @Array_Footer
							else
								header = @Array_Line
							end
							val = (" " * max_length) + header + val
							puts(val)
						}
					end
				end
			else
				puts(name + @Terminator + src.to_s)
			end
		}
	end
end

OptionParser.new do |opts|
	opts.banner = "Usage: build.rb [options]"
	
	opts.on("-n", "--name NAME", "Build Name [required]") { |v| $BuildOptions.name = v }
	opts.on("-t", "--target {#{build_target_str()}}", "Build Target [required]") { |v| $BuildOptions.target = v.downcase }
	opts.on("-r", "--root PATH", "Root Path") { |v| $BuildOptions.root = File.expand_path(normalize_path(v)) }
	opts.on("-I", "--intermediate PATH", "Intermediate Path [required]") { |v| $BuildOptions.intermediate = File.expand_path(normalize_path(v)) }
	opts.on("-o", "--output PATH", "Output File [required]") { |v| $BuildOptions.output = File.expand_path(normalize_path(v)) }
	opts.on("-d", "--deps PATH", "Output Dependencies") { |v| $BuildOptions.output_deps << File.expand_path(normalize_path(v)) }
	opts.on("-w", "--workers NUM", "Number of Worker Threads") { |v| $BuildOptions.threads = v.to_i }
	opts.on("-s", "--source PATH", "Source Path [required]") { |v| $BuildOptions.src_dirs << File.expand_path(normalize_path(v)) }
	opts.on("-v", "--verbose", "Show Diagnostic Data") { $BuildOptions.verbose = true }
	opts.on(nil, "--env ENV", "Specify the environment.") { |v| $environment << v }
end.parse!

config_error = $BuildOptions.validate
if (config_error != nil)
	raise ArgumentError.new(config_error)
end