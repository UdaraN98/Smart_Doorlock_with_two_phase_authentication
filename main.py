import sklearn
import numpy as np
import micromlgen
import pandas as pd
from sklearn.model_selection import train_test_split

from sklearn.ensemble import RandomForestClassifier
from micromlgen import port

data = pd.read_csv("datav2.csv")

X = data.drop(["Target"], axis = 1)

Y = data["Target"]




classifier = RandomForestClassifier(n_estimators=14, max_depth=10).fit(X, Y)

from sklearn.metrics import accuracy_score,f1_score
y_pre = classifier.predict(X)
print(accuracy_score(Y, y_pre))
print(f1_score(Y,y_pre))




c_code = port(classifier, classmap={0: "Not Allowed", 1: "Allowed"})
print(c_code)