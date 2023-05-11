import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler,OneHotEncoder
from sklearn.compose import ColumnTransformer

from sklearn import preprocessing


def process_list(myList):
    cat_column=np.array(myList)[:,3]
    print(cat_column)
    (unique, counts) = np.unique(np.array(myList)[:,3], return_counts=True)


    categorical_features = [3]
    categorical_transformer = OneHotEncoder(handle_unknown='ignore')

    preprocessor = ColumnTransformer(
        transformers=[('cat', categorical_transformer, categorical_features)], remainder='passp.shapthrough')
    #preprocessor = ColumnTransformer([('encoder', OneHotEncoder(), [3])], remainder='passthrough') 
    try:
        list_prep= preprocessor.fit_transform(myList)  
        print(list_prep)
        data = np.array(list_prep, dtype = np.float) 
        print(data)
    except ValueError as e:
        data_array=np.array(myList)
        unique = np.unique(data_array[:,3], return_counts=False)
        for cat in unique:
            print("------%s-----"%cat)
            current_array_indices=np.where(data_array[:,3]==cat)
            short_array=np.array(data_array[current_array_indices][:,:2],dtype=float)
            print(short_array)
            #TODO make cluster for each categories

        print (e)
    

    try:
           db = DBSCAN(eps=0.1, min_samples=2).fit(data)

           print (db.labels_)
    except Exception as e:
        print('ERROR during point clustering DBSCAN e:'+str(e))
        return {}




list=[['0.478925979158', '1.72041366053', '0.338162953405', 'pitcher'], ['0.0', '0.0', '0.0', 'legoduplo'], ['0.839994215872', '1.39460499135', '0.220023868548', 'pitcher'], ['1.43720197384', '1.74155567275', '0.515741799004', 'pitcher'], ['0.0', '0.0', '0.0', 'legoduplo'], ['3.10619888936', '3.18450739502', '0.000960778115468', 'coloredwoodblock'], ['3.13818368201', '3.49937216121', '0.000840265950091', 'coloredwoodblock'], ['1.83351236984', '5.08687828065', '0.861111093251', 'pottedmeat'], ['1.51749251286', '4.82177696414', '0.748751834135', 'sugar'], ['1.13746645355', '5.0060511129', '0.895427254161', 'pottedmeat'], ['0.858438474374', '4.62266843231', '0.961419217456', 'mustard'], ['0.691892589903', '4.95993932903', '0.862440519674', 'pottedmeat'], ['0.406098451559', '4.49834130458', '0.895938725193', 'mustard'], ['0.12812369009', '3.89425594954', '0.534817657272', 'legoduplo'], ['-0.27551997642', '4.73159381279', '0.600139205634', 'gelatin'], ['-0.583274241587', '4.63919945311', '0.600312986393', 'pottedmeat'], ['-0.725278029395', '4.54877949488', '0.530165151059', 'mustard'], ['-0.930979771043', '4.17856720669', '0.298943007401', 'lemon'], ['0.0', '0.0', '0.0', 'gelatin'], ['0.0', '0.0', '0.0', 'pottedmeat'], ['0.324886017928', '4.84101028613', '0.482104467322', 'tuna'], ['0.0', '0.0', '0.0', 'mustard'], ['-0.15773771022', '4.70698538593', '0.121250146812', 'strawberry'], ['-0.512578612166', '4.60848387188', '0.399463721501', 'lemon'], ['0.0', '0.0', '0.0', 'mustard'], ['0.113144736685', '4.78243481513', '0.233974019509', 'legoduplo'], ['-0.327620506584', '4.66011023016', '0.0769996761656', 'legoduplo'], ['0.438153607422', '4.8728203368', '0.347743386239', 'legoduplo'], ['-0.27024133607', '5.62588573594', '0.181816546111', 'legoduplo'], ['0.3229969383', '4.84099333885', '0.350572543268', 'legoduplo'], ['-0.655597595775', '3.44368888534', '0.390338001539', 'pitcher'], ['-0.240414552757', '5.08268149248', '0.311368182172', 'pitcher'], ['2.66803955614', '4.44257795974', '0.500825322086', 'legoduplo'], ['3.61838012352', '4.3264846375', '0.4755975006', 'cup'], ['3.62616664945', '3.72937549471', '0.000888229785055', 'legoduplo']]
list_simple=[[1,0,0,'A'],[1.05,0,0,'A'],[0,0,0,'B'],[0,0,0,'C'],[0,0,0,'D'],[0,0,0,'E'],[0,0,0,'F'],[0,0,0,'G'],[0,0,0,'H'],[0,0,0,'I'],[0,0,0,'J'],[0,0,0,'K']]
list_simple_working=[[1,0,0,'A'],[1.05,0,0,'A'],[1.05,0,0,'B'],[0,0,0,'C'],[0,0,0,'D'],[0,0,0,'E'],[0,0,0,'F'],[0,0,0,'G'],[0,0,0,'H'],[0,0,0,'I'],[0,0,0,'J']]


process_list(list)




#columnTransformer = ColsklearnumnTransformer([('encoder', OneHotEncoder(), [3])], remainder='passthrough') 
#Apply modification on the given data
#data = np.array(columnTransformer.fit_transform(coordlist_cropped), dtype = np.float) 