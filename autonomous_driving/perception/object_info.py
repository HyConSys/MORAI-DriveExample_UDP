#!/usr/bin/env python
# -*- coding: utf-8 -*-
from ..localization.point import Point


class ObjectInfo:
    def __init__(self, x, y, velocity, object_type, name="", length=-1, width=-1, hight=-1):
        """Simulator 에서 얻어지는 object 정보를 담는 data class"""
        self.position = Point(x, y)
        self.velocity = velocity
        self.type = object_type  # 0: person / 1: Sur-vehicl / 2: Object
        self.name = name
        self.length = length
        self.width = width
        self.hight = hight

    def __str__(self):
        return "%s(%r)" % (self.__class__, self.__dict__)
