

morse = (
    # Буквы
    ('A', '.-',   '11101110'),
    ('B', '-...', '00111110'),
    ('C', '-.-.', '10011100'),
    ('D', '-..',  '01111010'),
    ('E', '.',    '10011110'),
    ('F', '..-.', '10001110'),
    ('G', '--.',  '10111100'),
    ('H', '....', '01101110'),
    ('I', '..',   '01100001'),
    ('J', '.---', '01111000'),
    ('K', '-.-',  '10101111'),
    ('L', '.-..', '00011100'),
    ('M', '--',   '11101101'),
    ('N', '-.',   '00101010'),
    ('O', '---',  '11111101'),
    ('P', '.--.', '11001110'),
    ('Q', '--.-', '11100110'),
    ('R', '.-.',  '00001010'),
    ('S', '...',  '10110111'),
    ('T', '-',    '00011111'),
    ('U', '..-',  '01111100'),
    ('V', '...-', '00111000'),
    ('W', '.--',  '01010101'),
    ('X', '-..-', '01101111'),
    ('Y', '-.--', '01110110'),
    ('Z', '--..', '11011011'),

    # Цифры
    ('0', '-----', '11111100'),
    ('1', '.----', '01100000'),
    ('2', '..---', '11011010'),
    ('3', '...--', '11110010'),
    ('4', '....-', '01100110'),
    ('5', '.....', '10110110'),
    ('6', '-....', '10111110'),
    ('7', '--...', '11100000'),
    ('8', '---..', '11111110'),
    ('9', '----.', '11110110'),

    # Знаки препинания
    ('.', '.-.-.-', '00000001'),
    (',', '--..--', '00001000'),
    ('?', '..--..', '11001010'),
    ('!', '-.-.--', '01100010'),
)


morse = list(map(lambda x: (x[0], '1'+x[1].replace('-','1').replace('.','0'), x[2] ),morse))

morse = sorted(morse, key=lambda x: x[0])


print('//generated arrays')

print('uint8_t sym_to_code_key[] = {',end='')
for sym, code, segment in morse:
	print(f"{ord(sym)},", end='')
print('};\n')


print('uint8_t sym_to_code_value[] = {')
for sym, code, segment in morse:
	num = '0b'+code
	print(f"    {num}, //{sym}")
print('};\n')

print('uint8_t sym_to_segment_value[] = {')
print('//    ABCDEFG.')
for sym, code, segment in morse:
    print(f"    {'0b'+segment}, //{sym}")
print('};\n')



morse = sorted(morse, key=lambda x: int(x[1],2))


print('uint8_t code_to_sym_key[] = {')
for sym, code, segment in morse:
	num = '0b'+code
	print(f"    {num}, //{sym}")
print('};\n')

print('char code_to_sym_value[] = {',end='')
for sym, code, segment in morse:
	print(f"'{sym}',", end='')
print('};\n')



print(f'const uint8_t ALPHABET_LEN = {len(morse)};')