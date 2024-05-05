def findDistanceToBlock(height):
        if (height < 26):
            d = ((51-46) * (height - 24) / (24-26)) + 51
        elif (height < 27):
            d = ((3) * (height - 26) / (26-27)) + 46
        elif (height < 28):
            d = ((3) * (height - 27) / (27-28)) + 43
        elif (height < 33):
            d = ((3) * (height - 28) / (28-33)) + 40
        elif (height < 36):
            d = ((3) * (height - 33) / (33-36)) + 37
        elif (height < 39):
            d = ((3) * (height - 36) / (36-39)) + 34
        elif (height < 43):
            d = ((3) * (height - 39) / (39-43)) + 31
        elif (height < 51):
            d = ((3) * (height - 43) / (43-51)) + 28
        elif (height < 56):
            d = ((3) * (height - 51) / (51-56)) + 25
        elif (height < 65):
            d = ((3) * (height - 56) / (56-65)) + 22
        elif (height < 77):
            d = ((3) * (height - 65) / (65-77)) + 19
        elif (height < 90):
            d = ((3) * (height - 77) / (77-90)) + 16
        elif (height < 119):
            d = ((3) * (height - 90) / (90-119)) + 13
        elif (height < 185):
            d = ((3) * (height - 119) / (119-185)) + 10

        else:
            return -1

        return d


print(findDistanceToBlock(int(input("Enter height: "))))