import pandas as pd

df = pd.read_csv("./accel.csv")

print(df)
print(df.mean())
print(df.std())
print(df.var())