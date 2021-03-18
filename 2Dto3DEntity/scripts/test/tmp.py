import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler,OneHotEncoder
from sklearn.compose import ColumnTransformer


coord_list=[(0, 0, 5, 'toto'), (0, 0.5, 5, 'toto'), (0.5, 0, 5, 'toto'), (0.5, 0, 4.3, 'toto'), (3, 2, 0, 'car'), (2.5, 1.5, 0, 'car'), (3.2, 1.75, 0, 'car'), (10, 0, 5, 'bottle'), (10.5, 1, 5, 'bottle'), (9.5, 0.5, 5, 'bottle')]
coord_list=[[0, 0, 5, 'toto'], [0, 0.5, 5, 'toto'], [0.5, 0, 5, 'toto'], [0.5, 0, 4.3, 'toto'], [3, 2, 0, 'car'], [2.5, 1.5, 0, 'car'], [3.2, 1.75, 0, 'car'], [10, 0, 5, 'bottle'], [10.5, 1, 5, 'bottle'], [9.5, 0.5, 5, 'bottle']]
data= np.asarray([[0, 0, 5, 'toto'], [0, 0.5, 5, 'toto'], [0.5, 0, 5, 'toto'], [0.5, 0, 4.3, 'toto'], [3, 2, 0, 'car'], [2.5, 1.5, 0, 'car'], [3.2, 1.75, 0, 'car'], [10, 0, 5, 'bottle'], [10.5, 1, 5, 'bottle'], [9.5, 0.5, 5, 'bottle']])
category_data = np.unique(np.choose(3, data.T))
category_data2 =['toto','car','bottle']



columnTransformer = ColumnTransformer([('encoder', OneHotEncoder(), [3])], remainder='passthrough') 
  
data = np.array(columnTransformer.fit_transform(coord_list), dtype = np.float) 






enc2 = OneHotEncoder(categories=[category_data2])
enc2.fit(coord_list)
X=enc2.transform(coord_list)



enc = OneHotEncoder(handle_unknown='ignore')

enc.fit(data)
X=enc.transform(data)
enc2.fit(category_data)
X=enc2.transform(coord_list)

db = DBSCAN(eps=1, min_samples=3).fit(X)

print(db.labels_)