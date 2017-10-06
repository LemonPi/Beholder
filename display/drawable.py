class Drawable(object):
    def add_to_window(self, window):
        self.uid = window.add_drawable(self)

    def remove_from_window(self, window):
        window.remove_drawable(self.uid)

    def draw(self, window):
        pass
