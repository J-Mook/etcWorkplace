import tkinter as tk
from PIL import ImageGrab

root = tk.Tk()
entry_x = tk.Entry(root, width=10 )
entry_x.grid(row=0, column=1)
entry_y = tk.Entry(root, width=10 )
entry_y.grid(row=0, column=2)
entry_hex = tk.Entry(root, width=10 )
entry_hex.grid(row=1, column=2)
label_hex = tk.Label(root, width=10 )
label_hex.grid(row=1, column=1)

def motion(event):
    # coor_x, coor_y = event.x, event.y
    
    coor_x, coor_y = root.winfo_pointerx(), root.winfo_pointery()
    # coor_x, coor_y = root.winfo_pointerxy()
    # print('{}, {}'.format(coor_x, coor_y))
    
    entry_x.delete(0,"end")
    entry_x.insert(0,coor_x)
    entry_y.delete(0,"end")
    entry_y.insert(0,coor_y)

def grab(event):
    coor_x, coor_y = root.winfo_pointerx(), root.winfo_pointery()
    mouse_screen = ImageGrab.grab(bbox=(coor_x, coor_y, coor_x+1, coor_y+1))
    r,g,b,_ = mouse_screen.getpixel((0,0))
    # mouse_screen = ImageGrab.grab(include_layered_windows=True)
    # r,g,b,_ = mouse_screen.getpixel((coor_x, coor_y))
    print(r, g, b)
    hex = rgb_to_hex((r,g,b))
    # print(hex)
    entry_hex.delete(0,"end")
    entry_hex.insert(0,hex)
    label_hex.config(bg = hex)
    print("{},{} | {} | {}".format(coor_x, coor_y, mouse_screen.getpixel((0,0)), hex))
    

def rgb_to_hex(rgb):
    return '#%02x%02x%02x' % rgb

root.bind('<Key-f>', grab)
root.bind('<Motion>', motion)
root.wm_attributes("-topmost", 1)
root.mainloop()

# import pyautogui
# from PIL import ImageGrab

# while 1:
#     screen = ImageGrab.grab() # 화면 캡쳐
#     print(screen.getpixel(pyautogui.position()))