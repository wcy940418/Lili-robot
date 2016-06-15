import json

class PoseError(StandardError):
    pass

class PoseServer(object):
    def __init__(self):
        # self._isLoaded = False
        self._dict = {}
        self._filename = None

    @property
    def dict(self):
        return self._dict

    @property
    def filename(self):
        return self._filename 

    def load(self, s):
        """Given the filename of a json file `s`, load the data into a dict
        """
        self._filename = s
        # self._isLoaded = True
        with open(s, 'r') as f:
            self._dict = json.load(f)

    def create(self, s):
        """Given the filename of a json file `s`, create an empty json file with
        name `s`
        """
        with open(s, 'w') as f:
            json.dump({}, f)

    def save(self):
        """Saves the current dict to the loaded file
        """
        if self._filename:
            with open(self._filename, 'w') as f:
                json.dump(self._dict, f, sort_keys=True, indent=4,
                          separators=(',', ': '))
        else:
            raise PoseError("Please load map first")

    def append(self, name, data):
        """Given the name of a pose `name` and the pose data `data`, save the
        information into the loaded json file
        """
        if self._filename:
            if name not in self._dict:
                self._dict[name] = data
                self.save()
            else:
                raise PoseError("Key %s already exists" % name)

    def findall(self, name):
        """Given the name of a pose `name`, check whether it exists in the dict.
        Return the value if it does, raise an error otherwise
        """
        if name in self._dict:
            return self._dict[name]
        else:
            raise PoseError("%s is not in database" % name)

    # TODO: ???
    def find(self, _name, _label):
        if _name in self._dict:
            return self._dict[_name][_label]
        else :
            raise PoseError("%s is not in database" % _name)

    def delete(self, name):
        """Given the name of a pose `name`, delete its entry in the dict
        """
        if name in self._dict:
            self._dict.pop(name)
            self.save()
        else:
            raise PoseError("%s is not in database" % _name)

    # TODO: ???
    def revise(self, name, label, data):
        """
        """
        if name in self._dict:
            self._dict[name][label] = data
        else:
            raise PoseError("%s is not in database" % _name)
