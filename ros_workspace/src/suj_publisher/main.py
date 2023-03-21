from textual.app import App, ComposeResult
from textual.widgets import Button, Header, Footer, Static, Input, Label
from textual.containers import Container

class JointName(Static):
    def _(): pass
    
class ArmName(Static):
    def _(): pass
    
    # def compose(self) -> ComposeResult:
    #     yield Static("Arm: " + self.name)

class JointValue(Input):
    def _(): pass
    
    # def __init__(self): self.value="0.0"
    

class Joint(Static):
    
    def compose(self) -> ComposeResult:
        yield JointName(self.name)
        yield JointValue()

    
class Arm(Static):
    """A Arm widget."""
    # def __init__(self):
    #     self.arm_name = self.kwargs["arm_name"]
    #     self.arm_color = self.kwargs["arm_color"]
    
    def compose(self) -> ComposeResult:
        """Create child widgets of a stopwatch."""
        # yield Static("Arm: " + self.name)
        yield ArmName(self.name)
        yield Container(
            Joint("Joint0",id="j0"),
            Joint("Joint1",id="j1"),
            Joint("Joint2",id="j2"),
            Joint("Joint3",id="j3"),
            Joint("Joint4",id="j4"),
            Joint("Joint5",id="j5")
        )
        
class ThreeArms(Static):
    """A Arm widget."""

    def compose(self) -> ComposeResult:
        """Create child widgets of a stopwatch."""
        yield Arm("PSM1")
        yield Arm("PSM2")
        yield Arm("ECM")
        
class SUJPublisherApp(App):
    """A Textual app to manage stopwatches."""
    CSS_PATH = "style.css"
    BINDINGS = [("d", "toggle_dark", "Toggle dark mode")]

    def compose(self) -> ComposeResult:
        """Create child widgets for the app."""
        yield Header()
        yield Footer()
        yield ThreeArms()

    def action_toggle_dark(self) -> None:
        """An action to toggle dark mode."""
        self.dark = not self.dark


if __name__ == "__main__":
    app = SUJPublisherApp()
    app.run()