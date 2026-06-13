num = input('数字10：')
try:
    a = int(num)
    num = 10/a
    print('jisuanjieguo',num)
except Exception as e:
    print("",e)
