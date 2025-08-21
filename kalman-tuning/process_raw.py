import pandas as pd

df = pd.read_csv("./accel.csv")
df["acc_angle"] *= 180 / 3.14159265

print(df)
print("MEAN:")
print(df.mean())

print("STD:")
print(df.std())


print("VAR:")
print(df.var())