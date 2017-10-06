class Drawable(object):
    def add_to_window(self, screen):
        self.uid = screen.add_drawable(self)

    def remove_from_screen(self, screen):
        screen.remove_drawable(self.uid)

    def draw(self, screen):
        pass
