import pytest
from homewrk2.map import inflate, met2index, index2met, OBSTACLES, RESOLUTION

obs = [(35,70, 25, 90), (85, 120, 75, 130), (130, 165, 20, 85), (145, 180, 120, 165)]

def test_inflate():
    print (OBSTACLES)
    new = inflate(obstacles=obs, saftey_rad=4)
    print(new)
    return new
#test = test_inflate()

def test_index2met():
    assert index2met(17) == 35.0

def test_met2index():
    assert met2index(35) == 17
