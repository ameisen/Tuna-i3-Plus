require 'pathname'

# Normalize paths, needed because NT likes '\' and UNIX (and Ruby) like '//'
def normalize_path(path)
	if (path == "")
		return "."
	end
	path.tr!("\\", "/")
	# Remove duplicate elements of '/', as they may mess up some code.
	loop do
		new_path = path.tr("//", "/")
		if (new_path == path)
			return path
		end
		path = new_path
	end
end

$TAB = "  "

# Other helper functions
def puttab(str, tabs = 1)
	return ($TAB * tabs) + str
end
def puttabs(str, tabs = 1)
	puts puttab(str, tabs)
end

# Parse arguments
$build_targets = ["build", "clean", "rebuild", "install"];
def build_target_str
	out = ""
	$build_targets.each { |target|
		out += target.downcase
		out += " "
	}
	return out.chop;
end

def best_path (path, root = $BuildOptions.root)
	absolute_path = File.expand_path(normalize_path(path))
	
	# Check if the prefix is different on systems that, well, have a prefix.
	if (absolute_path.length >= 2 && root.length >= 2)
		prefix_path = absolute_path[0..2].downcase
		prefix_root = root[0..2].downcase
		if (prefix_path != prefix_root && prefix_path[0] != '/' && prefix_root != '/')
			# We cannot really alter this path otherwise.
			return absolute_path
		end
	end
	
	relative_path = Pathname.new(absolute_path).relative_path_from(Pathname.new(root)).to_s;
	
	if !(relative_path.include? "/")
		if (relative_path != ".")
			relative_path = "./" + relative_path
		end
	end
	
	if (absolute_path.length <= relative_path.length)
		return absolute_path
	else
		return relative_path
	end
end

def clean_paths(paths, root = $BuildOptions.root)
	# Validate that we are passing an array.
	if !(paths.kind_of? Array)
		raise ArgumentError.new("clean_paths must take an array object.")
	end
	# Validate that all elements are strings.
	paths.each { |path|
		if !(path.kind_of? String)
			raise ArgumentError.new("clean_paths array argument must be an array of stringable objects")
		end
	}

	# First we convert to absolute paths, so we can remove duplicate elements
	paths.collect! { |path|
		File.expand_path(path)
	}
	# Then we remove duplicates.
	paths.uniq!
	# Then we convert them back to best paths.
	paths.collect! { |path|
		best_path(path)
	}
	# Now we sort them by length, which should give good path locality.
	paths.sort_by! {|path| path.length}
end