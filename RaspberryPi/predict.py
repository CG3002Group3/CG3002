from sklearn.externals import joblib
import numpy as np
import time

move = ["WTF", "Wavehand", "Jump", "Frontback", "Turnclap", "Window"]
clf = joblib.load('walk_or_run_0.1.pkl')



# Sample only must be an array of 18 floats: 6 means, 6 variances, 6 medians
walk_sample = [0.458476,-1.004446,-0.088374,-0.053638,-0.031348,-0.07992, 0.021267429,0.047860016,0.015594766,0.162634999,0.282407261,3.881078688,0.45645,-0.98135,-0.0728,-0.0201,-0.0515,0.20695]
walk_sample_np = np.array(walk_sample).reshape(1,-1)

prev_time = time.time()

run_sample = [-1.084362,0.037612,0.028544,0.086112,-0.263366,0.157954, 1.488288678,0.250297367,0.038472106,3.508335569,1.911543066,3.598217251,-0.68995,0.02045,0.01135,0.26055,-0.44345,-0.45825]
run_sample_np = np.array(run_sample).reshape(1,-1)

print("Predicted walk sample: " + move[int(clf.predict(walk_sample_np)[0])])
print("Predicted run sample: " + move[int(clf.predict(run_sample_np)[0])])
print("Time taken: " + str(time.time() - prev_time))