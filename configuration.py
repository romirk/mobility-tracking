from re import S
from typing import Union


class Configuration:
    def __init__(self, config: dict):
        for key in config.keys():
            setattr(self, key, config[key])

    def __repr__(self):
        return str(self.config)


class ConfigurationBuilder:
    def __init__(self, config: Union[dict, list] = {}):
        self.config = config
        if type(config) == dict:
            for key in config.keys():
                setattr(self, key, lambda v: ConfigurationBuilder._setter(self, key, v))
        elif type(config) == list:
            for key in config:
                setattr(self, key, lambda v: ConfigurationBuilder._setter(self, key, v))
        else:
            raise TypeError("config must be dict or list")

    @staticmethod
    def _setter(self, key, value):
        self.config[key] = value
        return self

    def build(self):
        return Configuration(self.config)

    def __getattr__(self, name):
        def setter(value):
            self.config[name] = value
            return self

        return setter

    def __repr__(self):
        return str(self.config)
