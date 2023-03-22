# Copyright (C) 2023 Alberto Rota
# 
# This file is part of suj_publisher.
# 
# suj_publisher is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# suj_publisher is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with suj_publisher.  If not, see <http://www.gnu.org/licenses/>.

# Composition
from textual.app import App, ComposeResult
# Widgets
from textual.widgets import Button, Header, Footer, Static, Input, Label
# Containers
from textual.containers import Container, Vertical, Horizontal


# APP CLASS
class SUJPublisherApp(App):
    
    CSS_PATH = "sujpublisher.css"
    BINDINGS = [
        ("q", "quit_app", "Quit"), # Press Q to quit
        ("p", "publish_suj", "Publish SUJs"), # Press P to publish
        ("r", "reset_suj", "Reset to Default"), # Press R to reset
    ]

    # App Composition
    def compose(self) -> ComposeResult:
        yield Header()
        yield Footer()
        with Horizontal(id="threearms"):
            # with Vertical(id="left-pane"):
            #     for number in range(15):
            #         yield Static(f"Vertical layout, child {number}")
            with Vertical(id="psm1"):
                yield Static("Text")
                yield Static("Goes")
                yield Static("Here")
                yield Static("!")
            with Vertical(id="ecm"):
                yield Static("Text")
                yield Static("Goes")
                yield Static("Here")
                yield Static("!")
            with Vertical(id="psm2"):
                yield Static("Text")
                yield Static("Goes")
                yield Static("Here")
                yield Static("!")
            # with Container(id="bottom-right"):
            #     yield Static("This")
            #     yield Static("panel")
            #     yield Static("is")
            #     yield Static("using")
            #     yield Static("grid layout!", id="bottom-right-final")

        
    def action_quit_app(self) -> None:
        app.quit()

# -------------------------------------------------------------------------- #
if __name__ == "__main__":
    app = SUJPublisherApp()
    app.run()