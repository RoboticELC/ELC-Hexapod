import shutil

class Directory:
    def __init__(self, source_path, destination_path):
        self.source = source_path
        self.destination = destination_path

    def copy_directory(self):
        try:
            print("hi")
            shutil.copy(self.source, self.destination)
            print("File copied successfully!")

        except Exception as e:
            print("hello")
            print("Error:", e)
            print("Potential cause: ", e.__cause__)

    def move_directory(self):
        try:
            print("hi")
            shutil.move(self.source, self.destination)
            print("File moved successfully!")
        except Exception as e:
            print("hello")
            print("Error:", e)
            print("Potential cause: ", e.__cause__)


if __name__ == "__main__":
    dir = Directory(source_path="C:/Users/hilly/Downloads/libraries/library", destination_path="C:/Users/hilly/OneDrive/Documents/GitHub/ELC-Hexapod/packages")
    # dir = Directory(source_path="C:/Users/hilly/Downloads/Skematik1.jpg", destination_path="C:/Users/hilly/OneDrive/Documents/GitHub/ELC-Hexapod")
    dir.move_directory()