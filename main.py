import lidars
import utility
import contour
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import lidarsDrv as lid
import pylab
import warnings

def disp_menu():
    print('')
    print('----MENU----') 
    if drv.areConnected == 0:
        print('1: Connect to lidars')
        print('2: Plot datas')
        print('3: Compute circumference')
    else:
        print('1: Disconnect to lidars')
        print('2: Start motors')
        print('3: Stop motors')
        print('4: Launch scan')
        print('5: Plot datas')
        print('6: Compute circumference')
    print('q: Quit')

    
def execute_actions():
    choice = input('    Waiting for input: ')
    if choice == 'q' or choice == 'Q':
        if drv.areConnected == 1:
            drv.disconnect()
        return True
    
    if drv.areConnected == 0:
        if choice == '1':
            # Connect to lidar
            drv.connect()
        elif choice == '2':
            lidarsSet.read_datas_files()
            lidarsSet.plot_raw_datas()
            print('\nClose all figures to continue...\n')
            plt.show(block=True)
        elif choice == '3':
            # Compute circumference
            compute_algo_circumference()
        else:
            print('        Wrong input!')
    else:
        if choice == '1':
            # Disconnect to lidar
            drv.disconnect()
        elif choice == '2':
            if drv.check_link_state() == 0:
                drv.start_motors()        
        elif choice == '3':
            if drv.check_link_state() == 0:
                drv.stop_motors()       
        elif choice == '4':
            height = int(input('    Please enter patient\'s height in cm: '))
            if height > 0:
                # Data scanning
                if drv.check_link_state() == 0:
                    drv.scan_datas(height*10)
            else:
                print('    Incorrect height!')
        elif choice == '5':
            lidarsSet.read_datas_files()
            lidarsSet.plot_raw_datas()
            print('\nClose all figures to continue...\n')
            plt.show(block=True)
        elif choice == '6':
            compute_algo_circumference()
        else:
            print('        Wrong input!')
    return False            
        
        
def compute_algo_circumference():
    # Read lidars datas
    lidarsSet.read_datas_files()

    # Plot raw datas
    lidarsSet.plot_raw_datas()

    # Compute raw datas into merged datas
    lidarsSet.compute_raw_datas()
    
    if len(utility.mergedPointsXY) > 0: 
        # Compute contour
        contour.contour()

        # Compute circumference
        utility.compute_circumference()

        # Show the plots
        print('\nClose all figures to continue...\n')
        plt.show(block=True)

    else:
        print('    No data point has been read in the desired area!')


# Ignore warnings
warnings.simplefilter("ignore")
   
# Init lidars infos
lidarsSet = lidars.setOfLidars()

# Init lidars
drv = lid.driverLidars()

end = False
while end == False:
    disp_menu()
    end = execute_actions()

print("Program ended")



