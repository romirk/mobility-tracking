from typing import Union
from copy import deepcopy


class Configuration:
    def __init__(self, config: dict):
        self.config = config
        for key in config.keys():
            setattr(self, key, config[key])

    def __repr__(self):
        return str(self.config)


class ConfigurationBuilder:
    def __init__(self, config: Union[dict, list] = {}):
        self.config = config
        if type(config) == dict:
            for key in config.keys():
                setattr(self, key, self._setter(key))
        elif type(config) == list:
            for key in config:
                setattr(self, key, self._setter(key))
            self.config = {k:None for k in config}
        else:
            raise TypeError("config must be dict or list")

    def _setter(self, key):
        def _set(value):
            self.config[key] = value
            return self

        return _set

    def build(self):
        return Configuration(self.config)

    def __getattr__(self, name):
        def setter(value):
            self.config[name] = value
            return self

        return setter

    def __repr__(self):
        return str(self.config)
