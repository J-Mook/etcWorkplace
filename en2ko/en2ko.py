from tkinter import *

choseong_list = [char for char in "ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ"]
jungseong_list = [char for char in "ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ"]
jongseong_list = [char for char in " ㄱㄲㄳㄴㄵㄶㄷㄹㄺㄻㄼㄽㄾㄿㅀㅁㅂㅄㅅㅆㅇㅈㅊㅋㅌㅍㅎ"]
ko_dict = {'q':'ㅂ', 'Q':'ㅃ', 'w':'ㅈ', 'W':'ㅉ', 'e':'ㄷ', 'E':'ㄸ', 'r':'ㄱ', 'R':'ㄲ', 't':'ㅅ', 'T':'ㅆ', 'y':'ㅛ', 'u':'ㅕ', 'i':'ㅑ', 'o':'ㅐ', 'p':'ㅔ', 'a':'ㅁ', 's':'ㄴ', 'd':'ㅇ', 'f':'ㄹ', 'g':'ㅎ', 'h':'ㅗ', 'j':'ㅓ', 'k':'ㅏ', 'l':'ㅣ', 'z':'ㅋ', 'x':'ㅌ', 'c':'ㅊ', 'v':'ㅍ', 'b':'ㅠ', 'n':'ㅜ', 'm':'ㅡ', }

def main():
    
    tk = Tk()
    tk.title('한영 변환기')
    def click_btn():
        main_in = text1.get("1.0", END)
        text2.delete("1.0","end")
        text2.insert(END,en2ko(main_in))

    def return_btn(event):
        main_in = text1.get("1.0", END)
        print(main_in)
        text2.delete("1.0","end")
        text2.insert(END,en2ko(main_in))
    
    def clear():
        text1.delete("1.0",END)
        text2.delete("1.0",END)
    
    label1 = Label(tk,text='영어\n(입력)').grid(row=0, column=0,padx=10)
    label2 = Label(tk,text='한글\n(변환결과)').grid(row=1,column=0,padx=10)
    label3 = Label(tk,text='Made by j-mook').grid(row=2,column=1,padx=10, sticky='se')
    # 각 단위 입력받는 부분 만들기
    text1 = Text(tk)
    text2 = Text(tk)
    text1.grid(row=0,column=1,padx=5, pady=10, ipadx=20, ipady=0)
    text2.grid(row=1,column=1,padx=5, ipadx=20, ipady=0)

    btn1 = Button(tk,text='한/영 변환 [shift + 엔터]',command=click_btn).grid(row=2,column=1, pady=10)
    btn2 = Button(tk,text='초기화',command=clear).grid(row=2,column=0, pady=10)
    tk.bind("<Key>", return_btn)
    # tk.bind("<Shift-Return>", return_btn)
    tk.mainloop()

def en2ko(main_input):
    # convert en 2 ko
    ko_word = []
    for c in main_input:
        try:
            ko_word.append(ko_dict[c])
        except:
            ko_word.append(c)
    ko_word = list(''.join(ko_word)) + ['\n']
    
    # seperate by one letter
    words = []
    start = 0
    for i in range(1, len(ko_word)):
        if (i == len(ko_word)-1):
            words.append(ko_word[start:len(ko_word)])
        elif (ko_word[i] in choseong_list and ko_word[i+1] in jungseong_list) or (ko_word[i] not in choseong_list and ko_word[i] not in jungseong_list):
            words.append(ko_word[start:i])
            start = i
    print(words)
    
    # convert dubble letter
    for word in words:
        if len(word) > 2:
            if word[1] in jungseong_list and word[2] in jungseong_list:
                b = word[1]
                word[1] = make_jungseong_list(word[1:3])
                if (b != word[1]):
                    word.pop(2)
            if word[-1] in jongseong_list and word[-2] in jongseong_list:
                b = word[-2]
                word[2] = make_jongseong_list(word[-2:])
                if (b != word[-2]):
                    word.pop(-1)
    print(words)
    
    #combine each letter
    output_list = []
    for char in words:
        jongseong_index = 0
        if char[0] in choseong_list and char[1] in jungseong_list:
            choseong_index = choseong_list.index(char[0])
            jungseong_index = jungseong_list.index(char[1])
            if len(char) > 2 and char[2] in jongseong_list:
                jongseong_index = jongseong_list.index(char[2])
                char.pop(0)
            character_code = jongseong_index + 0xAC00 + (choseong_index * 21 * 28) + (jungseong_index * 28)
            output_list.append(chr(character_code))
            char.pop(0)
            char.pop(0)
        # else:
        while char:
            output_list.append(char.pop(0))

    print("{}\n\t||\n{}".format(main_input, ''.join(output_list)))
    return ''.join(output_list)

def make_jongseong_list(char_list):
    if char_list[0] == 'ㄱ' and char_list[1] == 'ㄱ':
        return "ㄲ"
    if char_list[0] == 'ㄱ' and char_list[1] == 'ㅅ':
        return "ㄳ"
    if char_list[0] == 'ㄴ' and char_list[1] == 'ㅈ':
        return "ㄵ"
    if char_list[0] == 'ㄴ' and char_list[1] == 'ㅎ':
        return "ㄶ"
    if char_list[0] == 'ㄹ' and char_list[1] == 'ㄱ':
        return "ㄺ"
    if char_list[0] == 'ㄹ' and char_list[1] == 'ㅁ':
        return "ㄻ"
    if char_list[0] == 'ㄹ' and char_list[1] == 'ㅂ':
        return "ㄼ"
    if char_list[0] == 'ㄹ' and char_list[1] == 'ㅅ':
        return "ㄽ"
    if char_list[0] == 'ㄹ' and char_list[1] == 'ㅌ':
        return "ㄾ"
    if char_list[0] == 'ㄹ' and char_list[1] == 'ㅍ':
        return "ㄿ"
    if char_list[0] == 'ㄹ' and char_list[1] == 'ㅎ':
        return "ㅀ"
    if char_list[0] == 'ㅂ' and char_list[1] == 'ㅅ':
        return "ㅄ"
    return char_list[0]

def make_jungseong_list(char_list):
    if char_list[0]=='ㅗ' and char_list[1] == 'ㅏ':
        return "ㅘ"
    if char_list[0]=='ㅗ' and char_list[1] == 'ㅐ':
        return "ㅙ"
    if char_list[0]=='ㅗ' and char_list[1] == 'ㅣ':
        return "ㅚ"
    if char_list[0]=='ㅜ' and char_list[1] == 'ㅓ':
        return "ㅝ"
    if char_list[0]=='ㅜ' and char_list[1] == 'ㅔ':
        return "ㅞ"
    if char_list[0]=='ㅜ' and char_list[1] == 'ㅣ':
        return "ㅟ"
    if char_list[0]=='ㅡ' and char_list[1] == 'ㅣ':
        return "ㅢ"
    return char_list[0]


main()