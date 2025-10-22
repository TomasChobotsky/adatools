from custom_dxl.CustomDXL import CustomDXL

object_dxls = CustomDXL(dxl_ids=[2, 3]) # Put your motor IDs from Dynamixel SDK
print(f"IDs: {object_dxls.getIDs()}")