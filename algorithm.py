class Algorithm(object):
    def __init__(self, G):
        raise NotImplementedError

    def setup(self, start, goal):
        raise NotImplementedError

    def solve(self):
        raise NotImplementedError

    def drive(self, policy, diverge_policy):
        raise NotImplementedError
