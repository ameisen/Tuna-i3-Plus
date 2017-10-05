$script_mtime = [$script_mtime, File.mtime(__FILE__).to_f].max

class AtomicInt
	@value
	@mutex
	
	def initialize (val = 0)
		@value = val
		@mutex = Mutex.new
	end
	
	def + (val)
		@mutex.synchronize do
			val = @value + val
		end
		return AtomicInt.new val
	end
	
	def - (val)
		@mutex.synchronize do
			val = @value - val
		end
		return AtomicInt.new val
	end
	
	def set (val)
		@mutex.synchronize do
			@value = val
		end
	end
	
	def get ()
		@mutex.synchronize do
			return @value
		end
	end
	
	def add! (val)
		@mutex.synchronize do
			@value += val
		end
		return self
	end
	
	def subtract! (val)
		@mutex.synchronize do
			@value -= val
		end
		return self
	end
	
	def fetch_increment! ()
		@mutex.synchronize do
			old_value = @value
			@value += 1
			return old_value
		end
	end
	
	def cas (compare, val)
		@mutex.synchronize do
			if (@value == compare)
				@value = val
				return [true, val]
			else
				return [false, @value]
			end
		end
	end
end