class DataCategory(object):
    def __init__(self, name):
        self.name = name

    def update(self, new_data):
        """Accepts new data for the data category."""
        raise NotImplementedError('Data category did not implement update!')

    def render(self):
        """Returns a PyGame Surface that can be blitted onto the window."""
        raise NotImplementedError('Data category did not implement draw!')
