#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
# @Time    : 2020/5/14 9:39
# @Author  : zlb
# @Email   : 15967924690@163.com
# @File    : mapping.py
# @Software: PyCharm
# import xlrd
# wb = xlrd.open_workbook('mapping.xlsx',encoding_override='utf-8')
# sheet1 = wb.sheet_by_name('Sheet1')

# dict_mapping = {}
# for row_num in range(2,sheet1.nrows):
#     row = sheet1.row_values(row_num)
#     row_safety = row[0]
#     row_time = row[1]
#     row_comfort = row[2]
#     row_local_planner = row[3]
#     row_global_planner = row[4]
#     requirement = row_safety +" " +  row_time + " " + row_comfort
#     dict_mapping[requirement] = row_global_planner + " + " +row_local_planner
# print(dict_mapping)
import os
os.environ['X_START'] ='1.0'
print('/`$START')
