# Composition
from textual.app import App, ComposeResult
# Widgets
from textual.widgets import Button, Header, Footer, Static, Input, Label
# Containers
from textual.containers import Container, Vertical, Horizontal


class Arm(Static):
        # Compostition of the app
    def compose(self) -> ComposeResult:
        yield Label("Name") #, id="armname")
        yield Label("J0n") #, id="jname")
        yield Label("J0v") #, id="jval")
        yield Label("J0n") #, id="jname")
        yield Label("J0v") #, id="jval")

# APP CLASS
class SUJPublisherApp(App):
    
    CSS_PATH = "sujpublisher.css"
    BINDINGS = [
        ("q", "quit_app", "Quit"), # Press Q to quit
        ("d", "toggle_dark", "Toggle dark mode"), # Press D to toggle dark mode
    ]
    DEFAULTS_PATH = "defaults.json"
    
    # App Composition
    def compose(self) -> ComposeResult:
        yield Header()
        yield Footer()
        yield Arm()
        yield Arm()
        yield Arm()

    def load_json(self, path: str) -> dict:
        with open(path) as f:
            return json.load(f)
    # Actions
    def action_toggle_dark(self) -> None:
        self.dark = not self.dark
        
    def action_quit_app(self) -> None:
        app.quit()

# -------------------------------------------------------------------------- #
if __name__ == "__main__":
    app = SUJPublisherApp()
    app.run()