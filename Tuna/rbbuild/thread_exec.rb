$script_mtime = [$script_mtime, File.mtime(__FILE__).to_f].max

require 'etc'
require_relative 'atomic_int.rb'

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
		
		mutex = Mutex.new
		mutex.lock
		@threads = Array.new(@num_threads)
		atomic_idx = AtomicInt.new
		(0 .. (@num_threads - 1)).each { |idx|
			@threads[idx] = Thread.new {
				mutex.lock
				mutex.unlock
				loop do
					work_idx = atomic_idx.fetch_increment!
					if (work_idx >= final_idx)
						break
					end
					functor.call(work_idx)
				end
			}
		}
		mutex.unlock
	end
	
	def join
		(0 .. (@num_threads - 1)).each { |idx|
			@threads[idx].join
		}
	end
end