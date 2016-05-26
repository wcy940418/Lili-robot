import json

class PoseError(StandardError):
	pass

class PoseServer(object):
	def __init__(self):
		self._isLoaded = False
		self._dict = {}
	def load(self, _s):
		self._loaded_file = _s
		self._isLoaded = True
		self._f = open(_s, 'r')
		self._dict = json.load(self._f)
		self._f.close()
	def create(self, _s):
		self._f = open(_s, 'w')
		dic = {}
		json.dump(dic, self._f)
		self._f.close()
	def save(self):
		if self._isLoaded:
			self._f = open(self._loaded_file, 'w')
			json.dump(self._dict, self._f)
			self._f.close()
		else:
			raise PoseError("Please load map first")
	def append(self, _name, _data):
		if self._isLoaded:
			self._dict[_name] = _data
			print self._loaded_file
			self._f = open(self._loaded_file, 'w')
			print self._dict
			json.dump(self._dict, self._f)
			self._f.close()
		else:
			raise PoseError("Please load map first")
	def fetchall(self):
		return self._dict
	def findall(self, _name):
		if _name in self._dict:
			return self._dict[_name]
		else:
			raise PoseError("%s is not in database" % _name)
	def find(self, _name, _label):
		if _name in self._dict:
			return self._dict[_name][_label]
		else :
			raise PoseError("%s is not in database" % _name)
	def delete(self,_name):
		if _name in self._dict:
			self._dict.pop(_name)
		else:
			raise PoseError("%s is not in database" % _name)
	def revise(self, _name, _label, _data):
		if _name in self._dict:
			self._dict[_name][_label] = _data
		else:
			raise PoseError("%s is not in database" % _name)

