def format_name(input: str):
    return input.replace(':','_').replace(' ','')

def rename_if_duplicate(input: str, in_dict: dict):
    count = 0
    new_name = input
    while in_dict.get(new_name) is not None:
        new_name = f"{input}_{count}"
        count += 1
    return new_name