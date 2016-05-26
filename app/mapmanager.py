#!/usr/bin/env python

import ImageFile
import os

class MapError(StandardError):
	pass

class MapManager(object):
	def __init__(self, maps_dir = "maps"):
		self._maps_dir = maps_dir
		get_path = os.path.abspath('.')
		_dir = os.listdir(get_path)
		if self._maps_dir in _dir:
			self._map_file_dir_path = os.path.join(get_path, self._maps_dir)
			MapManager.buildmaplib(self)			
		else :
			os.mkdir(os.path.join(get_path, self._maps_dir))

	def refreshmaplib(self):
		self._all_pgm = [os.path.splitext(x)[0] for x in os.listdir(self._map_file_dir_path) if os.path.isfile(os.path.join(self._map_file_dir_path, x)) and os.path.splitext(x)[1] == '.pgm']
		self._all_yaml = [os.path.splitext(x)[0]  for x in os.listdir(self._map_file_dir_path) if os.path.isfile(os.path.join(self._map_file_dir_path, x)) and os.path.splitext(x)[1] == '.yaml']
		self._all_bmp = [os.path.splitext(x)[0]  for x in os.listdir(self._map_file_dir_path) if os.path.isfile(os.path.join(self._map_file_dir_path, x)) and os.path.splitext(x)[1] == '.bmp']
		self._valid_maps = [x for x in self._all_pgm if x in self._all_yaml]
		self._has_bmp = [x for x in self._valid_maps if x in self._all_bmp]
		self._need_bmp = [x for x in self._valid_maps if not x in self._all_bmp]
	def buildmaplib(self):
		MapManager.refreshmaplib(self)
		if not self._valid_maps:
			print "Please create new map first"
		else :
			if self._need_bmp:
				for x in self._need_bmp:
					MapManager._pgm2bmp(self, x)
				MapManager.refreshmaplib(self)
			if self._need_bmp:
				for x in self._need_bmp:
					str = str + "x" + ".pgm"
					for i in range(len(self._valid_maps)):
						if x == self._valid_maps[i]:
							self._valid_maps.pop[i]
				raise MapError("Cannot generate all bmp for valid maps, please check: %s" % str)
	def loadmap4mapserver(self, _name):
		if _name in self._valid_maps:
			return  os.path.join(self._map_file_dir_path, _name + ".yaml")
		else :
			raise MapError("No such file in maplib")
	def getmappath(self, _name):
		if _name in self._valid_maps:
			return  os.path.join(self._map_file_dir_path, _name)
		else :
			raise MapError("No such file in maplib")
	def getmapdirpath(self):
		return self._map_file_dir_path
	def _pgm2bmp(self, _name):
		_file = os.path.join(self._map_file_dir_path, _name)
		fp = open(_file + ".pgm","rb")
		p = ImageFile.Parser()
		while True:
			s = fp.read(1024)
			if not s: break
			p.feed(s)
		Im = p.close()
		Im.save(_file + ".bmp")
	def getallvalidmap(self):
		return self._valid_maps
	def buildnewmap(self, _name):
		return os.path.join(self._map_file_dir_path, _name)

if __name__ == '__main__':
	try:
		map1 = MapManager()
		
	except:
		print("Cannot invoke map")
	try:
		print map1.loadmap4mapserver('floor1')
	except:
		pass
	try:
		print map1.loadmap4mapserver('floor7')
	except:
		pass
	print map1.getallvalidmap()
	print map1.buildnewmap('floor7')
	
	
