def invert_dictionary(dictionary):
    inv_map = {v: k for k, v in dictionary.items()}
    return inv_map



loc_to_marker = {

#x: 'A' thru 'H'
#y: 1 thru 8
('A',1) : 'ar_marker_1',
('B',1) : 'ar_marker_2',
('C',1) : 'ar_marker_3',
('D',1) : 'ar_marker_4',
('E',1) : 'ar_marker_5',
('F',1) : 'ar_marker_6',
('G',1) : 'ar_marker_7',
('H',1) : 'ar_marker_8',
('A',2) : 'ar_marker_9',
('B',2) : 'ar_marker_10',
('C',2) : 'ar_marker_11',
('D',2) : 'ar_marker_12',
('E',2) : 'ar_marker_13',
('F',2) : 'ar_marker_14',
('G',2) : 'ar_marker_15',
('H',2) : 'ar_marker_16',
('A',3) : 'ar_marker_17',
('B',3) : 'ar_marker_18',
('C',3) : 'ar_marker_19',
('D',3) : 'ar_marker_20',
('E',3) : 'ar_marker_21',
('F',3) : 'ar_marker_22',
('G',3) : 'ar_marker_23',
('H',3) : 'ar_marker_24',
('A',4) : 'ar_marker_25',
('B',4) : 'ar_marker_26',
('C',4) : 'ar_marker_27',
('D',4) : 'ar_marker_28',
('E',4) : 'ar_marker_29',
('F',4) : 'ar_marker_30',
('G',4) : 'ar_marker_31',
('H',4) : 'ar_marker_32',
('A',5) : 'ar_marker_33',
('B',5) : 'ar_marker_34',
('C',5) : 'ar_marker_35',
('D',5) : 'ar_marker_36',
('E',5) : 'ar_marker_37',
('F',5) : 'ar_marker_38',
('G',5) : 'ar_marker_39',
('H',5) : 'ar_marker_40',
('A',6) : 'ar_marker_41',
('B',6) : 'ar_marker_42',
('C',6) : 'ar_marker_43',
('D',6) : 'ar_marker_44',
('E',6) : 'ar_marker_45',
('F',6) : 'ar_marker_46',
('G',6) : 'ar_marker_47',
('H',6) : 'ar_marker_48',
('A',7) : 'ar_marker_49',
('B',7) : 'ar_marker_50',
('C',7) : 'ar_marker_51',
('D',7) : 'ar_marker_52',
('E',7) : 'ar_marker_53',
('F',7) : 'ar_marker_54',
('G',7) : 'ar_marker_55',
('H',7) : 'ar_marker_56',
('A',8) : 'ar_marker_57',
('B',8) : 'ar_marker_58',
('C',8) : 'ar_marker_59',
('D',8) : 'ar_marker_60',
('E',8) : 'ar_marker_61',
('F',8) : 'ar_marker_62',
('G',8) : 'ar_marker_63',
('H',8) : 'ar_marker_64'
}

marker_to_loc = invert_dictionary(loc_to_marker)

list_of_board_markers = loc_to_marker.values()