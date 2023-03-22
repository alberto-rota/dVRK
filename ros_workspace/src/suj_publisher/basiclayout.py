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
        ("p", "publish_suj", "Quit"), # Press Q to quit
        ("r", "reset_suj", "Quit"), # Press Q to quit
        ("d", "default_suj", "Quit"), # Press Q to quit
    ]

    # App Composition
    def compose(self) -> ComposeResult:
        yield Header()
        yield Footer()

        
    def action_quit_app(self) -> None:
        app.quit()

# -------------------------------------------------------------------------- #
if __name__ == "__main__":
    app = SUJPublisherApp()
    app.run()