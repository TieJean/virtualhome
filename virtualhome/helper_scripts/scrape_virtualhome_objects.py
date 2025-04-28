import argparse
import requests
from bs4 import BeautifulSoup
import pandas as pd

def fetch_virtualhome_objects(url):
    """
    Fetch and parse the VirtualHome object list from the given URL.
    Returns a cleaned pandas DataFrame.
    """
    # Fetch page
    response = requests.get(url)
    soup = BeautifulSoup(response.text, 'html.parser')

    # Find table
    table = soup.find('table')

    # Parse headers
    headers = [header.text.strip() for header in table.find_all('th')]

    # Parse rows
    rows = []
    current_object_name = None

    for row in table.find_all('tr')[1:]:  # skip header
        cells = row.find_all('td')
        cell_texts = [cell.text.strip() for cell in cells]

        if len(cell_texts) == len(headers):
            current_object_name = cell_texts[0]
        else:
            # fill in missing Object Name
            cell_texts = [current_object_name] + cell_texts

        # Clean encoding artifacts
        cleaned_row = [text.replace('â', 'y') for text in cell_texts]
        rows.append(cleaned_row)

    # Create DataFrame
    df = pd.DataFrame(rows, columns=headers)

    # Fill down missing Object Names if any
    df['Object Name'] = df['Object Name'].replace('', pd.NA).ffill()

    return df

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fetch and parse VirtualHome object list.")
    parser.add_argument('--url', type=str,
                        default="http://virtual-home.org/documentation/master/kb/objects.html",
                        help="URL to fetch the VirtualHome object list from.")
    parser.add_argument('--savepath', type=str, default=None,
                        help="Path to save the cleaned CSV. If not specified, will not save.")

    args = parser.parse_args()

    df = fetch_virtualhome_objects(args.url)

    if args.savepath:
        df.to_csv(args.savepath, index=False)
        print(f"✅ Successfully saved cleaned table to '{args.savepath}'")
    else:
        print("✅ Successfully fetched and parsed VirtualHome object list (no file saved).")
