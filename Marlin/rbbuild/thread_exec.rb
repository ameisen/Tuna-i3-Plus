require 'etc'
require File.expand_path(File.dirname(__FILE__) + '/atomic_int.rb')

class Threader
	@num_threads
	@threads
	
	def initialize(functor, final_idx, num_threads = $BuildOptions.threads)
		if (num_threads == nil)
			num_cpu = Etc.nprocessors
			@num_threads = num_cpu
		else
			@num_threads = num_threads
		end
		
		@threads = Array.new(@num_threads)
		atomic_idx = AtomicInt.new
		(0 .. (@num_threads - 1)).each { |idx|
			@threads[idx] = Thread.new {
				loop do
					work_idx = atomic_idx.fetch_increment!
					if (work_idx >= final_idx)
						break
					end
					functor.call(work_idx)
				end
			}
		}
	end
	
	def join
		(0 .. (@num_threads - 1)).each { |idx|
			@threads[idx].join
		}
	end
end