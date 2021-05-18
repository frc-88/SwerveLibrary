from dearpygui.core import *
from dearpygui.simple import *


def save_callback(sender, data):
    print("Save Clicked")


def main():
    with window("Example Window"):
        add_text("Hello, world")
        add_button("Save", callback=save_callback)
        add_input_text("string", default_value="Quick brown fox")
        add_slider_float("float", default_value=0.273, max_value=1)

    start_dearpygui()


if __name__ == "__main__":
    main()
