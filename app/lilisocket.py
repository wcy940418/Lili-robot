import socket

class LiliSocket(socket.socket):
	def __enter__(self):
		return self

	def __exit__(self, exception_type, exception_value, traceback):
		self.shutdown(2) # shutdown receiving and sending
		self.close()
