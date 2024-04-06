import shutil
def move_directory(source_path, destination_path):
    try:
        print("hi")
        shutil.copy(source_path, destination_path)
        print("JPG file copied successfully!")

    except Exception as e:
        print("hello")
        print("Error:", e)
        print("Potential cause: ", e.__cause__)

if __name__ == "__main__":
    move_directory(source_path="C:/Users/hilly/Downloads/Skematik1.jpg", destination_path="C:/Users/hilly/OneDrive/Documents/GitHub/ELC-Hexapod")